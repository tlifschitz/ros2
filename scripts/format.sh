#!/bin/bash

# ROS2 Code Formatting Script using uncrustify
# This script formats C++ code according to ROS2 style guidelines

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if uncrustify is installed
if ! command -v uncrustify &> /dev/null; then
    print_error "uncrustify is not installed. Please install it first:"
    echo "  Ubuntu/Debian: sudo apt-get install uncrustify"
    echo "  macOS: brew install uncrustify"
    exit 1
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"

# Default ROS2 uncrustify configuration
UNCRUSTIFY_CONFIG="$PROJECT_ROOT/.uncrustify.cfg"

# Function to format a single file
format_file() {
    local file="$1"
    local backup_file="${file}.backup"
    
    # Create backup
    cp "$file" "$backup_file"
    
    # Format the file
    if uncrustify -c "$UNCRUSTIFY_CONFIG" --replace --no-backup "$file" 2>/dev/null; then
        if cmp -s "$file" "$backup_file"; then
            # No changes made
            rm "$backup_file"
            echo "  ✓ $file (no changes)"
        else
            # Changes were made
            rm "$backup_file"
            echo "  ✓ $file (formatted)"
        fi
    else
        # Restore backup on error
        mv "$backup_file" "$file"
        print_error "Failed to format $file"
        return 1
    fi
}

# Function to format files in directory
format_directory() {
    local dir="$1"
    local count=0
    
    print_info "Formatting C++ files in $dir..."

    # Find all C++ files and store in array
    local files=()
    while IFS= read -r -d '' file; do
        files+=("$file")
    done < <(find "$dir" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" \) -print0)

    # Format each file
    for file in "${files[@]}"; do
        if [[ -f "$file" ]]; then
            format_file "$file"
            ((count++))
        else
            print_warning "Skipping non-regular file: $file"
        fi
    done

    print_info "Formatted $count files in $dir"
}

# Main script logic
cd "$PROJECT_ROOT"

# Parse command line arguments
if [ $# -eq 0 ]; then
    # Format the entire src directory
    if [ -d "src" ]; then
        format_directory "src"
    else
        print_error "No src directory found. Please run this script from your ROS2 workspace root."
        exit 1
    fi
elif [ $# -eq 1 ]; then
    if [ -f "$1" ]; then
        # Format a single file
        print_info "Formatting single file: $1"
        format_file "$1"
    elif [ -d "$1" ]; then
        # Format a directory
        format_directory "$1"
    else
        print_error "File or directory not found: $1"
        exit 1
    fi
else
    # Format multiple files/directories
    for target in "$@"; do
        if [ -f "$target" ]; then
            print_info "Formatting file: $target"
            format_file "$target"
        elif [ -d "$target" ]; then
            format_directory "$target"
        else
            print_warning "Skipping non-existent file/directory: $target"
        fi
    done
fi

print_info "Code formatting complete!"
print_info "Uncrustify configuration: $UNCRUSTIFY_CONFIG"