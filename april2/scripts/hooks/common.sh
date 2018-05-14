# Shared functions for all checks

# Print warning
warning() {
    action=$1
    message=$2
    echo "WARNING : " $message
    echo "Continuing" $action
}

# Generate non-blocking warning
warning_with_response() {
    action=$1
    message=$2
    exec < /dev/tty
    echo "WARNING : " $message
    echo "Do you still want to continue? (y/n)"
    read -n 1 response
    echo ""
    if [ "$response" == "n" ]; then
        echo "Aborting" $action
        exit 1
    fi
    echo "Continuing" $action
}

# Generate error message and stop action
error() {
    action=$1
    message=$2
    echo "ERROR : " $message
    echo "Aborting" $action
    exit 1
}
