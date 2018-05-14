# Stops commit if binaries are present

if git diff --cached --numstat | grep -e '^-' >/dev/null; then
    warning_with_response $ACTION "Commit would add binary files:
    `git diff --cached --numstat | grep -e '^-' | cut -f3-`"
fi
