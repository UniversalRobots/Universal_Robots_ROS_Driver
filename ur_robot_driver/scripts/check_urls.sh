#!/bin/bash

set -e

#Find URLs in code:
urls=$(grep -oP "(http|ftp|https):\/\/([a-zA-Z0-9_-]+(?:(?:\.[a-zA-Z0-9_-]+)+))([a-zA-Z0-9_.,@?^=%&:\/~+#-]*[a-zA-Z0-9_@?^=%&\/~+#-])?" "$@")

fail_counter=0

for item in $urls; do
#     echo $item  
    filename=$(echo "$item" | cut -d':' -f1)
    url=$(echo "$item" | cut -d':' -f2-)
#     echo "Checking $url from file $filename"
    if ! curl --head --silent --fail "$url" 2>&1 > /dev/null; then
        echo "Invalid link in file $filename: $url"
        ((fail_counter=fail_counter+1))
    else
        echo "$url ok"
    fi
done

exit $fail_counter
