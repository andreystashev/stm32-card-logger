#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT HUP INT TERM
${CC:-cc} -std=c99 -Wall -Wextra -Werror -ICore/Inc tests/test_packet_buffer.c -o "$build_dir/test_packet_buffer"
${CC:-cc} -std=c99 -Wall -Wextra -Werror -Itests/stubs tests/test_diskio.c -o "$build_dir/test_diskio"
${CC:-cc} -std=c99 -Wall -Wextra -Werror -Itests/stubs tests/test_sd.c -o "$build_dir/test_sd"
"$build_dir/test_packet_buffer"
"$build_dir/test_diskio"
"$build_dir/test_sd"
