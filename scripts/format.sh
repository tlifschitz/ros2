#!/bin/bash

# ROS2 Code Formatting Script using ament_uncrustify
# This script formats C++ code according to ROS2 style guidelines
# Uses the same formatter as CI/colcon test for consistency

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

# Check if ament_uncrustify is available
if ! command -v ament_uncrustify &> /dev/null; then
    print_warning "ament_uncrustify is not available in this environment."
    print_info "To use ament_uncrustify (recommended for CI compatibility):"
    echo "  1. Source ROS2 setup: source /opt/ros/humble/setup.bash"
    echo "  2. Or run in a ROS2 container/environment"
    echo ""
    print_info "Alternative: Install and use regular uncrustify:"
    echo "  Ubuntu/Debian: sudo apt-get install uncrustify"
    echo "  macOS: brew install uncrustify"
    echo ""
    
    # Check if regular uncrustify is available as fallback
    if command -v uncrustify &> /dev/null; then
        print_warning "Falling back to regular uncrustify (may not match CI exactly)"
        USE_AMENT_UNCRUSTIFY=false
    else
        print_error "Neither ament_uncrustify nor uncrustify is available."
        exit 1
    fi
else
    USE_AMENT_UNCRUSTIFY=true
fi

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Function to format a single file
format_file() {
    local file="$1"
    local backup_file="${file}.backup"
    
    # Create backup
    cp "$file" "$backup_file"
    
    # Format the file
    local format_success=false
    if [[ "$USE_AMENT_UNCRUSTIFY" == "true" ]]; then
        if ament_uncrustify --reformat "$file" 2>/dev/null; then
            format_success=true
        fi
    else
        # Fallback to regular uncrustify with basic ROS2 style
        if uncrustify --replace --no-backup -l CPP \
            --set indent_columns=2 \
            --set indent_with_tabs=0 \
            --set sp_inside_paren=remove \
            --set sp_arith=add \
            --set sp_assign=add \
            --set sp_bool=add \
            --set sp_compare=add \
            --set sp_after_comma=add \
            --set sp_before_comma=remove \
            --set nl_if_brace=add \
            --set nl_for_brace=add \
            --set nl_while_brace=add \
            --set nl_else_brace=add \
            --set mod_full_brace_if=add \
            --set mod_full_brace_for=add \
            --set mod_full_brace_while=add \
            "$file" 2>/dev/null; then
            format_success=true
        fi
    fi
    
    if [[ "$format_success" == "true" ]]; then
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
    
    print_info "Formatting C++ files in $dir using ament_uncrustify..."

    # Use ament_uncrustify to format the entire directory
    if ament_uncrustify --reformat "$dir" 2>/dev/null; then
        # Count the files that would be processed
        local files=()
        while IFS= read -r -d '' file; do
            files+=("$file")
        done < <(find "$dir" -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.cc" -o -name "*.hh" -o -name "*.c" -o -name "*.h" \) -print0)
        
        count=${#files[@]}
        
        for file in "${files[@]}"; do
            echo "  ✓ $file (processed)"
        done
        
        print_info "Formatted $count files in $dir"
    else
        print_error "Failed to format directory $dir"
        return 1
    fi
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
print_info "Using ament_uncrustify (same as CI/colcon test)"