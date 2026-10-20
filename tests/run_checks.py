#!/usr/bin/env python3
"""One offline check entry, with a 300s cumulative execution budget per budget-state file."""
import argparse
import ast
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from urllib.parse import unquote
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def static_checks():
    """Check current documentation links, project sources and mirrored public files once."""
    for path in ROOT.rglob('*.md'):
        relative = path.relative_to(ROOT)
        if any(part.startswith('.') for part in relative.parts):
            continue
        text = path.read_text(encoding='utf-8')
        for target in re.findall(r'\]\(([^)]+)\)', text):
            target = target.strip('<>').split('#', 1)[0]
            if not target or re.match(r'^[a-zA-Z]+:', target):
                continue
            resolved = path.parent / unquote(target)
            assert resolved.exists(), f'Broken link: {relative} -> {target}'
    shared = ['HARDWARE/DFCom.h', 'HARDWARE/DFCom_Tx.c', 'HARDWARE/DFCom_Rx.c',
              'HARDWARE/DFCom_Print.c', 'SYSTEM/usart/usart.c', 'SYSTEM/usart/usart.h', 'USER/main.c']
    for relative in shared:
        assert (ROOT/'DFCom_Example'/relative).read_bytes() == (ROOT/'DFCom_PatchOnly'/relative).read_bytes(), relative
    for variant in ('DFCom_Example',):
        project = ROOT/variant/'USER/Template.uvprojx'
        tree = ET.parse(project)
        assert tree.findtext('.//useUlib') == '1', 'Keil printf target must enable MicroLIB'
        for element in tree.findall('.//FilePath'):
            source = project.parent / element.text.replace('\\', '/')
            if variant == 'DFCom_Example':
                assert source.is_file(), f'Missing Keil source: {source}'
        assert not any('beta_examples' in element.text or 'nuedc' in element.text
                       for element in tree.findall('.//FileName')), 'Superseded route still in project'
    for path in (ROOT/'ros2').rglob('*.py'):
        if not any(part.startswith('.') for part in path.relative_to(ROOT).parts):
            ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    for path in (ROOT/'ros2').rglob('package.xml'):
        ET.parse(path)
    subprocess.run(['git', 'diff', '--check'], cwd=ROOT, check=True, capture_output=True, text=True)
    print('Current Markdown links, Keil references, XML, Python syntax and STM32 mirror passed')


def run_process(command, seconds):
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1',
               PYTHONPATH=str(ROOT/'ros2/dcaron_bridge'),
               DFCOM_SANITIZE='1')
    process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, start_new_session=os.name != 'nt')
    try:
        output, _ = process.communicate(timeout=seconds)
    except subprocess.TimeoutExpired:
        if os.name == 'nt':
            subprocess.run(['taskkill', '/F', '/T', '/PID', str(process.pid)], capture_output=True)
        else:
            os.killpg(process.pid, signal.SIGKILL)
        process.communicate()
        raise TimeoutError('Cumulative 300s check budget exhausted; process group terminated')
    if process.returncode:
        raise RuntimeError(f'Command failed ({process.returncode}): {command}\n{output}')
    # Keep the summary compact; skipped ROS tests remain visible.
    lines = output.strip().splitlines()
    print('\n'.join(lines[-5:]))


def main():
    if sys.version_info < (3, 10):
        raise SystemExit('Use Python >=3.10 with PyYAML; the system Python may be older.')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', nargs='+', choices=['static', 'slots', 'queue', 'demo', 'arduino', 'python'])
    parser.add_argument('--python-pattern', default='test_*.py',
                        help='Target only a failed Python test file during repair')
    parser.add_argument('--budget-state', type=Path,
                        help='Reuse the same state file for targeted repair checks; never reset it within one delivery')
    options = parser.parse_args()
    state = {'spent_seconds': 0.0, 'results': {}}
    if options.budget_state and options.budget_state.exists():
        state = json.loads(options.budget_state.read_text())
    commands = {
        'static': None,
        'slots': ['sh', 'tests/host/run_move_slots_tests.sh'],
        'queue': ['sh', 'tests/host/run_queue_tests.sh'],
        'demo': ['sh', 'tests/host/run_demo_tests.sh'],
        'arduino': ['sh', 'tests/arduino/run_arduino_tests.sh'],
        'python': [sys.executable, '-m', 'unittest', 'discover', '-s',
                   'ros2/dcaron_bridge/test', '-p', options.python_pattern, '-v'],
    }
    failed = False
    for name in options.only or commands:
        remaining = 300 - state['spent_seconds']
        if remaining <= 0:
            print('STOP: cumulative 300s check budget exhausted')
            failed = True
            break
        print(f'[{name}]', flush=True)
        start = time.monotonic()
        try:
            if name == 'static':
                # Isolate static checks as a bounded process too.
                run_process([sys.executable, str(Path(__file__)), '--internal-static'], remaining)
            else:
                run_process(commands[name], remaining)
            state['results'][name] = 'passed'
        except (RuntimeError, TimeoutError) as error:
            print(error)
            state['results'][name] = 'failed'
            failed = True
        finally:
            state['spent_seconds'] += time.monotonic() - start
            if options.budget_state:
                options.budget_state.write_text(json.dumps(state, indent=2) + '\n')
    print(json.dumps(state, ensure_ascii=False, indent=2))
    print('Scope: offline only; Keil target build, ROS runtime and hardware were not validated here.')
    return 1 if failed else 0


if __name__ == '__main__':
    if sys.argv[1:] == ['--internal-static']:
        static_checks()
    else:
        raise SystemExit(main())
