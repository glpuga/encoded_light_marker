#/bin/bash
# Copyright (2022) Gerardo Puga
# Distributed under the MIT License (http://opensource.org/licenses/MIT)

FILE_REGEX_PATTERN='\.\/.*\.\(cpp\|hpp\|cc\|cxx\)'
CLANG_OPTIONS='-style=file -i --sort-includes'

find . \
    -regex $FILE_REGEX_PATTERN \
    -exec clang-format $CLANG_OPTIONS {} \;

exit 0
