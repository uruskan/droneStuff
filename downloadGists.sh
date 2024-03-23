#!/bin/bash

while true; do
    read -p "Enter the URL of the gist (or 'q' to quit): " gist_url
    if [ "$gist_url" == "q" ]; then
        echo "Exiting..."
        break
    fi
    echo "Cloning gist from $gist_url..."
    git clone $gist_url
done

