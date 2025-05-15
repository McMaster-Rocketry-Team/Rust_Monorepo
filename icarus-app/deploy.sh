#! /bin/bash

yarn build

# Function to handle cleanup
cleanup() {
    echo "Stopping all processes..."
    kill $(jobs -p) 2>/dev/null
    exit
}

# Set up trap for Ctrl+C
trap cleanup INT TERM

# Start both processes in background
yarn preview &
ssh -R 80:localhost:16000 nokey@localhost.run &

# Wait for both processes
wait