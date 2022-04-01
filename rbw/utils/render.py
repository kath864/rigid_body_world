import os
import sys
import json
import shlex
import argparse
import subprocess
from collections.abc import Iterable

from .encoders import NpEncoder

#######################################################################
# Interface
#######################################################################

# TODO document this
def render(**kwargs):
    """ Subprocess call to blender
    """
    out = ''
    if 'out' in kwargs:
        out += kwargs['out']
    if not os.path.isdir(out):
        os.mkdir(out)

    if isinstance(kwargs['trace'], str):
        t_path = kwargs.pop('trace')
    else:
        t_path = os.path.join(out, 'trace.json')
        with open(t_path, 'w') as temp:
            json.dump(kwargs.pop('trace'), temp,
                    cls = NpEncoder)

    if isinstance(kwargs['scene'], str):
        src_path = kwargs.pop('scene')
    else:
        src_path = os.path.join(out, 'scene.json')
        with open(src_path, 'w') as temp:
            json.dump(kwargs.pop('scene'), temp,
                    cls = NpEncoder)

    if 'exec' in kwargs:
        blend_exec = kwargs.pop('exec')
    else:
        blend_exec = 'blender'

    if 'blend' in kwargs:
        blend_file = kwargs.pop('blend')
    else:
        raise ValueError("Must provide blend file, blend=\"\"")

    if 'render' in kwargs:
        render_path = kwargs.pop('render')
    else:
        raise ValueError("Must provide render file, render=\"\"")

    if 'threads' in kwargs:
        threads = kwargs.pop('threads')
    else:
        threads = len(os.sched_getaffinity(0))

    _cmd = cmd.format(blend_exec, blend_file, render_path, threads)
    _cmd = shlex.split(_cmd)
    _cmd += make_args(kwargs)
    _cmd += ['--trace', t_path]
    _cmd += ['--scene', src_path]
    sys.stdout.flush()
    p = subprocess.run(_cmd)

#######################################################################
# Helpers
#######################################################################

# takes the blend file and the bpy script
cmd = '{0!s} --verbose 2 -noaudio --background {1!s} -P {2!s} -t {3:d}'

def make_args(args_d):
    cmd = ['--', '--save_world']
    for k,v in args_d.items():
        if v is None:
            cmd += ['--{0!s}'.format(k),]
        elif isinstance(v, str) or not isinstance(v, Iterable):
            cmd += ['--{0!s}'.format(k), str(v)]
        else :
            cmd += ['--{0!s}'.format(k),
                    *[str(e) for e in v]]
    return cmd
