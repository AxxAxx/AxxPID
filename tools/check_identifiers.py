#!/usr/bin/env python3
"""Reject identifiers that the C and C++ standards reserve to the implementation.

C reserves any identifier with a leading underscore at file scope. C++ goes
further and reserves every identifier containing a doubled underscore anywhere,
and these sources are compiled by C++ toolchains as well, so neither form may
appear. The compiler's own predefined macros are the one exception, since
testing for them is the whole point of writing them.

Run from the repository root:  python3 tools/check_identifiers.py
"""

import pathlib
import re
import sys

ALLOWED = re.compile(
    r"^__(FILE|LINE|DATE|TIME|func|VA_ARGS|cplusplus|STDC\w*|"
    r"GNUC\w*|clang\w*|has_\w+|attribute)(__)?$")

IDENTIFIER = re.compile(r"_*[A-Za-z_][A-Za-z0-9_]*")


def strip_comments(text):
    """Remove comments, so prose *about* reserved identifiers does not trip."""
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def main():
    root = pathlib.Path(__file__).resolve().parent.parent
    sources = sorted((root / "include").rglob("*.h"))
    sources += sorted((root / "src").rglob("*.c"))

    if not sources:
        print("no sources found - run this from the repository root")
        return 1

    bad = []
    for path in sources:
        text = strip_comments(path.read_text(encoding="utf-8"))
        for number, line in enumerate(text.splitlines(), 1):
            for identifier in IDENTIFIER.findall(line):
                if ALLOWED.match(identifier):
                    continue
                if identifier.startswith("_") or "__" in identifier:
                    bad.append("%s:%d: %s"
                               % (path.relative_to(root), number, identifier))

    for entry in bad:
        print("::error::reserved identifier -> " + entry)

    if bad:
        print("%d reserved identifier(s) found" % len(bad))
        return 1

    print("clean: %d files, no reserved identifiers" % len(sources))
    return 0


if __name__ == "__main__":
    sys.exit(main())
