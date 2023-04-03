#!/usr/bin/env bash

# Current script converts all the "//!< ..." doxygen inline comments into
# /**...*/ comments placed in the previous line.
# This runs on the provided file

if ! [[ $# -eq 1 ]]; then
	printf "./Usage convert_inline_doxygen_comments.sh <file_to_work_on>\n"
	printf "Exiting...\n"
	exit 1
fi

f=$1

# Modify all //!< ... comments to /** ... */ and place them in the previous line
# - Move the comment to the prevous line
# - Keep indentation level
# - Do not match comment lines (start with // or /*)
sed -i''  "/^\s\+[\/\*\|\/\/]/! s/^\(\s*\)\(.*\)\s*\/\/\!<\(.*\)$/\1\/\*\*\3 \*\/\\n\1\2/g" ${f}

# remove buggy "*/ */" expressions at the end of lines - print for verification
sed -n "/\*\/\s*\*\//p" ${f}
sed -i "s/\(\*\/\)\s*\*\//\1/g" ${f}

exit 0
