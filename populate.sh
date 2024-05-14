#!/bin/bash

readme="README.md"

if [ ! -f "$readme" ]; then
    echo "Error: README.md not found!"
    exit 1
fi

markdown_files=$(find results/ -type f -name "*.md")

if [ -z "$markdown_files" ]; then
    echo "No markdown files found in results/ directory."
    exit 0
fi

sed -i '/# Results/,/^#/d' "$readme"

echo "# Results" >> "$readme"
for file in $markdown_files; do
    echo "" >> "$readme"
    cat "$file" >> "$readme"
    echo "" >> "$readme"
done
