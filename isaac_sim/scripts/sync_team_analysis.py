"""Copy the team's judgement code (sensor Pi) into the twin so both judge with the same algorithm.

    python scripts/sync_team_analysis.py                  # heayoooon/... branch dev (default)
    python scripts/sync_team_analysis.py --branch juho/cog
    python scripts/sync_team_analysis.py --repo C:/path/to/clone --branch my/branch   (a local clone)

Copies pi/analysis/*.py and pi/acquisition/{config.py, frame.py, env.sh} into IsaacTwin/team/ and writes
team/TEAM_VERSION.json (repository, branch, commit, date). The twin (team_judge.py) imports only these
files; nothing of the hardware side (drivers, GPIO) is copied. Restart the twin after syncing.

--part display copies the display Pi's pure-numpy cargo box code (display/twin/server/{cargo_model,
stable_twin, stack_layers, surface_core}.py) into IsaacTwin/team_display/ for cargo_boxes.py; --part all does both.
"""
import argparse
import json
import shutil
import subprocess
import tempfile
import time
from pathlib import Path

REPO = 'https://github.com/heayoooon/2026ESWContest_mobility_JeokjaeJeokso.git'
ROOT = Path(__file__).resolve().parent.parent
DEST = ROOT / 'team'
FILES = ['pi/acquisition/config.py', 'pi/acquisition/frame.py', 'pi/acquisition/env.sh']
DISPLAY_DEST = ROOT / 'team_display'
DISPLAY_FILES = [f'display/twin/server/{n}.py' for n in ('cargo_model', 'stable_twin', 'stack_layers', 'surface_core')]


def git(*args, cwd=None):
    return subprocess.run(['git', *args], cwd=cwd, check=True, capture_output=True, text=True, encoding='utf-8').stdout.strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repo', default=REPO, help='git URL or local clone')
    ap.add_argument('--branch', default='dev')
    ap.add_argument('--part', choices=('analysis', 'display', 'all'), default='analysis')
    args = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        src = Path(tmp) / 'repo'
        if Path(args.repo).exists():
            git('clone', '-q', '--no-hardlinks', '--branch', args.branch, str(Path(args.repo).resolve()), str(src))
        else:
            git('clone', '-q', '--depth', '1', '--branch', args.branch, args.repo, str(src))
        commit = git('rev-parse', 'HEAD', cwd=src)
        subject = git('log', '-1', '--format=%s (%ci)', cwd=src)
        if args.part in ('display', 'all'):
            copy_display(src, args, commit, subject)
        if args.part == 'display':
            return
        names = sorted(str(p.relative_to(src)).replace('\\', '/') for p in (src / 'pi' / 'analysis').glob('*.py')) + FILES
        new = DEST.with_name('team.new')
        if new.exists():
            shutil.rmtree(new)
        for name in names:
            target = new / name.replace('pi/', '', 1)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src / name, target)
        (new / 'acquisition' / '__init__.py').touch()
        (new / 'analysis' / '__init__.py').touch()
        (new / 'TEAM_VERSION.json').write_text(json.dumps({
            'repo': args.repo, 'branch': args.branch, 'commit': commit, 'subject': subject,
            'synced': time.strftime('%Y-%m-%d %H:%M:%S'), 'files': names}, ensure_ascii=False, indent=2), encoding='utf-8')
        if DEST.exists():
            shutil.rmtree(DEST)
        new.rename(DEST)
    print(f'synced {len(names)} files from {args.repo} {args.branch} @ {commit[:7]}: {subject}')
    print('restart the twin to use them')


def copy_display(src, args, commit, subject):
    new = DISPLAY_DEST.with_name('team_display.new')
    if new.exists():
        shutil.rmtree(new)
    new.mkdir()
    for name in DISPLAY_FILES:
        shutil.copy2(src / name, new / Path(name).name)
    (new / 'TEAM_VERSION.json').write_text(json.dumps({
        'repo': args.repo, 'branch': args.branch, 'commit': commit, 'subject': subject,
        'synced': time.strftime('%Y-%m-%d %H:%M:%S'), 'files': DISPLAY_FILES}, ensure_ascii=False, indent=2), encoding='utf-8')
    if DISPLAY_DEST.exists():
        shutil.rmtree(DISPLAY_DEST)
    new.rename(DISPLAY_DEST)
    print(f'synced {len(DISPLAY_FILES)} display files from {args.repo} {args.branch} @ {commit[:7]}: {subject}')


if __name__ == '__main__':
    main()
