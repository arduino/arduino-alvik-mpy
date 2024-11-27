import sys

def reload_modules():
    to_be_reloaded = []

    for m in sys.modules:
        to_be_reloaded.append(m)
        del sys.modules[m]

    for m in to_be_reloaded:
        exec(f'import {m}')


reload_modules()