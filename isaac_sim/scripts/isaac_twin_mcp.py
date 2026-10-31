#!/usr/bin/env python3
"""MCP server (stdio) that lets your own Claude drive the JeokjaeJeokso Isaac Sim twin over the network.

No packages needed - Python 3.8+ standard library only.

  Claude Code:     claude mcp add isaac-twin -- python /path/isaac_twin_mcp.py --url http://100.109.1.106:8211
  Claude Desktop:  "mcpServers": {"isaac-twin": {"command": "python",
                       "args": ["/path/isaac_twin_mcp.py", "--url", "http://100.109.1.106:8211"]}}

The twin PC must be running Start-ForestTwin-Share.bat. Same network: http://192.168.0.17:8211.
"""
import argparse
import base64
import json
import os
import sys
import urllib.error
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument('--url', default=os.environ.get('TWIN_URL', 'http://100.109.1.106:8211'))
URL = ap.parse_args().url.rstrip('/')

INSTRUCTIONS = (
    'Tools for the JeokjaeJeokso cargo-monitoring digital twin (Isaac Sim, Hyundai XCIENT truck carrying the '
    'team\'s instrumented cargo plate: 4 load cells, 2 belt tension cells, IMU, D435i). Two modes: "measured" '
    'replays real sensor-Pi recordings, "physics" is a PhysX sandbox. Judgement levels 0 OK, 1 caution, '
    '2 warning, 3 danger for load, COG, cargo shift and belts. Call twin_commands first to see every action, '
    'slider range and recording; use twin_screenshot to look at the scene after changing something. '
    'Several people may use the twin at once - say what you changed.')

TOOLS = [
    {'name': 'twin_state', 'description': 'Current twin state: mode, recording, replay time, vehicle speed and lane, '
     'judgement levels and reasons, load cells, belts, IMU, COG, cargo shift, recording and display status.',
     'inputSchema': {'type': 'object', 'properties': {}}},
    {'name': 'twin_commands', 'description': 'List every action, every slider/toggle with its range, the recordings '
     'available for replay and the camera views.', 'inputSchema': {'type': 'object', 'properties': {}}},
    {'name': 'twin_command', 'description': 'Run one action on the twin (see twin_commands). Examples: '
     '{"action":"mode","value":"physics"}, {"action":"run","value":"20261015_212927"}, {"action":"go"}, '
     '{"action":"set","name":"grade","value":10}, {"action":"set","name":"straps","value":false}, '
     '{"action":"view","value":"Side"}, {"action":"record_start"}, {"action":"send_to_display"}.',
     'inputSchema': {'type': 'object', 'properties': {
         'action': {'type': 'string', 'description': 'action name from twin_commands'},
         'name': {'type': 'string', 'description': 'slider or toggle name for action "set"'},
         'value': {'description': 'value for the action (number, boolean or text)'}},
         'required': ['action']}},
    {'name': 'twin_screenshot', 'description': 'JPEG image of what the twin viewport shows right now.',
     'inputSchema': {'type': 'object', 'properties': {}}},
    {'name': 'list_recordings', 'description': 'Recordings saved with record_start/record_stop (newest first).',
     'inputSchema': {'type': 'object', 'properties': {}}},
    {'name': 'download_recording', 'description': 'Download one recording (zip with twin_log.csv, twin_log.jsonl, '
     'summary.json) to a folder on this computer.',
     'inputSchema': {'type': 'object', 'properties': {
         'name': {'type': 'string', 'description': 'recording name, e.g. 20261020_101500'},
         'folder': {'type': 'string', 'description': 'where to save (default: current folder)'}},
         'required': ['name']}},
]


def http(path, payload=None, raw=False, timeout=15):
    data = json.dumps(payload).encode() if payload is not None else None
    request = urllib.request.Request(URL + path, data=data, headers={'Content-Type': 'application/json'} if data else {})
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read()
    return body if raw else json.loads(body.decode('utf-8'))


def text(obj):
    return [{'type': 'text', 'text': obj if isinstance(obj, str) else json.dumps(obj, ensure_ascii=False, indent=1)}]


def call(name, args):
    if name == 'twin_state':
        return text(http('/twin/state'))
    if name == 'twin_commands':
        return text(http('/twin/commands'))
    if name == 'twin_command':
        return text(http('/twin/command', {k: args[k] for k in ('action', 'name', 'value') if k in args}))
    if name == 'twin_screenshot':
        jpeg = http('/twin/screenshot', raw=True, timeout=30)
        return [{'type': 'image', 'data': base64.b64encode(jpeg).decode(), 'mimeType': 'image/jpeg'}]
    if name == 'list_recordings':
        return text(http('/recordings/index.json'))
    if name == 'download_recording':
        rec = str(args['name'])
        folder = os.path.abspath(args.get('folder') or '.')
        os.makedirs(folder, exist_ok=True)
        target = os.path.join(folder, f'twin_{rec}.zip')
        with open(target, 'wb') as stream:
            stream.write(http(f'/recordings/{rec}.zip', raw=True, timeout=60))
        return text(f'saved {target}')
    raise ValueError(f'unknown tool {name}')


def send(message):
    sys.stdout.buffer.write(json.dumps(message, ensure_ascii=False).encode('utf-8') + b'\n')
    sys.stdout.buffer.flush()


def main():
    for line in sys.stdin.buffer:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        method, mid = msg.get('method'), msg.get('id')
        if mid is None:
            continue                       # notifications (initialized, cancelled) need no answer
        try:
            if method == 'initialize':
                version = (msg.get('params') or {}).get('protocolVersion') or '2025-06-18'
                result = {'protocolVersion': version, 'capabilities': {'tools': {}},
                          'serverInfo': {'name': 'isaac-twin', 'version': '1.0'}, 'instructions': INSTRUCTIONS}
            elif method == 'tools/list':
                result = {'tools': TOOLS}
            elif method == 'tools/call':
                params = msg.get('params') or {}
                try:
                    result = {'content': call(params.get('name'), params.get('arguments') or {}), 'isError': False}
                except urllib.error.HTTPError as error:
                    result = {'content': text(f'HTTP {error.code}: {error.read().decode("utf-8", "replace")}'), 'isError': True}
                except (urllib.error.URLError, OSError) as error:
                    result = {'content': text(f'cannot reach the twin at {URL} ({error}). Is Start-ForestTwin-Share.bat '
                                              'running on the twin PC and are you on the team Tailscale / same network?'),
                              'isError': True}
            elif method == 'ping':
                result = {}
            else:
                send({'jsonrpc': '2.0', 'id': mid, 'error': {'code': -32601, 'message': f'method {method} not found'}})
                continue
            send({'jsonrpc': '2.0', 'id': mid, 'result': result})
        except Exception as error:
            send({'jsonrpc': '2.0', 'id': mid, 'error': {'code': -32603, 'message': str(error)}})


if __name__ == '__main__':
    main()
