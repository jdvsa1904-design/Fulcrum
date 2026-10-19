import asyncio
import json
import os
import sys
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

ROOT = Path(__file__).resolve().parents[1]


async def main():
    rows = []
    params = StdioServerParameters(command=sys.executable, args=[str(ROOT / 'mcp' / 'fulcrum_mcp.py')], env={'FULCRUM_URL':os.environ.get('FULCRUM_URL','http://127.0.0.1:8790')})
    async with stdio_client(params) as streams:
        async with ClientSession(*streams) as session:
            await session.initialize()
            async def call(tool_name, **args):
                result = await session.call_tool(tool_name, args)
                text = '\n'.join(c.text for c in result.content if hasattr(c, 'text'))
                if getattr(result, 'is_error', False):
                    raise RuntimeError(tool_name + ': ' + text)
                try:
                    return json.loads(text)
                except ValueError:
                    return text

            def check(name, ok, **data):
                rows.append(dict(name=name, passed=bool(ok), **data))
                print(name, 'PASS' if ok else 'FAIL', flush=True)
                if not ok:
                    raise AssertionError(name + ': ' + str(data))

            tools = await session.list_tools()
            check('MCP guided tools registered', {'guided_motion', 'motion_quality'} <= {t.name for t in tools.tools})
            state = await call('open_project', name='FRT_fulcrum_v11.fulcrum.json')
            check('FRT baseline', state['frames'] == 324 and state['gate']['pass'], quality=state['guided']['quality'])
            await call('select', a=10, b=40)
            await call('eval_js', code="S.frame=25;api.showFrame(25);return true;")
            result = await call('guided_motion', action='weapon_grip')
            check('Weapon attachment and grips', len(result['state']['guided']['props']) == 1 and result['state']['pins'][-1]['kind'] == 'weapon')
            await call('eval_js', code="S.verificationBaseline={q:S.out.q.slice(),p:S.out.p.slice(),baseP:S.base.p.slice()};return true;")
            await call('guided_motion', action='inbetween', options={'strength': .35, 'preserveRoot': True})
            result = await call('eval_js', code="let outside=0,root=0;const nb=S.body.bones.length;for(let f=0;f<S.take.n;f++){for(let i=0;i<nb*4;i++)if(f<10||f>40)outside=Math.max(outside,Math.abs(S.out.q[f*nb*4+i]-S.verificationBaseline.q[f*nb*4+i]));for(let i=0;i<nb*3;i++)root=Math.max(root,Math.abs(S.base.p[f*nb*3+i]-S.verificationBaseline.baseP[f*nb*3+i]));}return {outside,root,quality:{max:S.motionQuality.constraintMaxCm,failed:S.motionQuality.failedConstraints}};")
            check('Inbetween root and outside range', result['outside'] < 1e-6 and result['root'] < 1e-5, **result)
            result = await call('guided_motion', action='reject')
            check('Preview rejection restores fixes', not any(x['type'] == 'inbetween' for x in result['state']['fixes']))
            result = await call('guided_motion', action='secondary', options={'secondaryStrength': .15})
            check('Secondary preview', result['state']['guided']['preview'] == 'Secondary motion')
            await call('guided_motion', action='accept')
            state = (await call('guided_motion', action='undo'))['state']
            check('Undo accepted change', not any(x['type'] == 'secondary' for x in state['fixes']))
            state = (await call('guided_motion', action='redo'))['state']
            check('Redo accepted change', any(x['type'] == 'secondary' for x in state['fixes']))
            await call('guided_motion', action='undo')
            await call('eval_js', code="S.selBone=api.idx('upperarm_r');return S.selBone;")
            state = (await call('guided_motion', action='suggest_pose'))['state']
            check('Guided pose suggestion preview', state['guided']['preview'] == 'Pose suggestion')
            result = await call('eval_js', code="let max=0;const nb=S.body.bones.length;for(let f=0;f<S.take.n;f++)if(f<13||f>37)for(let i=0;i<nb*4;i++)max=Math.max(max,Math.abs(S.out.q[f*nb*4+i]-S.verificationBaseline.q[f*nb*4+i]));return {outside:max};")
            check('Pose suggestion stays local', result['outside'] < 1e-5, **result)
            await call('guided_motion', action='reject')
            await call('guided_motion', action='physics')
            state = (await call('guided_motion', action='reject'))['state']
            check('Physics preview and reject', state['guided']['preview'] is None)
            await call('guided_motion', action='bookmark')
            await call('save_project', name='FRT_guided_verified')
            await call('open_project', name='FRT_guided_verified.fulcrum.json')
            state = await call('status')
            check('Saved weapon grips roundtrip', len(state['guided']['props']) == 1 and len(state['guided']['props'][0]['grips']['left']) == 16)
            quality = await call('motion_quality')
            check('Required contacts within tolerance', quality['failedConstraints'] == 0, maxCm=quality['constraintMaxCm'])
            path = ROOT / 'exports' / 'guided_integration_validation.json'
            path.write_text(json.dumps({'transport': 'MCP stdio -> Fulcrum HTTP bridge -> live browser', 'checks': rows}, indent=2))
            print(path, flush=True)


asyncio.run(main())
