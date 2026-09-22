"""Pull the real function/typedef text out of main.c by brace matching, so the
host test compiles the production source rather than a transcription of it."""
import re, sys

src = open(sys.argv[1], encoding='utf-8').read()
lines = src.split('\n')
out = []

def strip_code(ln, state):
    """Blank out string literals, char literals and comments so brace counting
    sees only real code. `state` carries block-comment continuation."""
    out = []
    i = 0
    while i < len(ln):
        c = ln[i]
        if state['comment']:
            if ln.startswith('*/', i):
                state['comment'] = False
                i += 2
            else:
                i += 1
            continue
        if ln.startswith('/*', i):
            state['comment'] = True
            i += 2
            continue
        if ln.startswith('//', i):
            break
        if c in '"\'':
            quote = c
            i += 1
            while i < len(ln):
                if ln[i] == '\\':
                    i += 2
                    continue
                if ln[i] == quote:
                    i += 1
                    break
                i += 1
            continue
        out.append(c)
        i += 1
    return ''.join(out)

def block_from(start_idx, opener='{', closer='}'):
    depth = 0
    started = False
    body = []
    state = {'comment': False}
    for i in range(start_idx, len(lines)):
        ln = lines[i]
        body.append(ln)
        code = strip_code(ln, state)
        depth += code.count(opener) - code.count(closer)
        if opener in code:
            started = True
        if started and depth == 0:
            return '\n'.join(body), i
    raise SystemExit('unbalanced block at line %d' % (start_idx + 1))

def grab_typedef(name):
    for i, ln in enumerate(lines):
        if ln.startswith('typedef struct {'):
            body, end = block_from(i)
            if body.rstrip().endswith('} %s;' % name):
                return body
    raise SystemExit('typedef %s not found' % name)

def grab_func(name):
    pat = re.compile(r'^(static\s+)?[A-Za-z_][\w \*]*\b%s\s*\(' % re.escape(name))
    for i, ln in enumerate(lines):
        if pat.match(ln) and not ln.rstrip().endswith(';'):
            body, end = block_from(i)
            return body
    raise SystemExit('function %s not found' % name)

for arg in sys.argv[2:]:
    kind, name = arg.split(':')
    out.append('/* --- extracted from main.c: %s --- */' % name)
    out.append(grab_typedef(name) if kind == 'type' else grab_func(name))
    out.append('')

print('\n'.join(out))
