#!/bin/bash

# Check if a commit message was provided as input
if [ -z "$1" ]; then
  echo "Error: Commit message is required."
  echo "Usage: ./git_push.sh \"your commit message\""
  exit 1
fi

# Add all changes
git add .

# Commit with the provided message
git commit -m "$1"

# Push to the main branch
git push origin main

# Success message
echo "Changes pushed to 'main' branch."

