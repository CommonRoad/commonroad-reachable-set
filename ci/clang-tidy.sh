#!/bin/bash

# set exit on error
# set -e

exit_flag=false

clang_tidy="clang-tidy"

target_branch="develop"

# get target branch
git fetch origin ${target_branch}

# Retrieve list of cpp-files that were changed in source branch with respect to target branch
filelist=($(git diff origin/${target_branch} --name-only | grep ".cpp"))


if [[ "${#filelist[@]}" -eq "0" ]]; then
    echo "==> No cpp files found"
    echo "==> clang-tidy has nothing to do, stop early"
    exit 0
else
    echo "==> Found ${#filelist[@]} cpp files"
    echo "==> ${filelist[*]}"
    echo "==> Let's start our clang-tidy check"
fi

echo
echo "clang-tidy checking changed files compared to target branch ${target_branch}"

# function to check if C++ file (based on suffix)
function checkCPP(){
    if [[ -f $1 ]] && [[ $1 == *.cpp ]]; then
        return 0
    fi
    return 1
}

$clang_tidy --version
echo


filesWithErrors=()

# check list of files
for f in ${filelist[*]}; do
    # check if .cpp file and in compilation DB
    if checkCPP $f && [[ -n $(grep $f build/compile_commands.json) ]]; then
        echo "Checking matching file ${f}"
        touch output.txt
        $clang_tidy -p=build ${f} --extra-arg=--cuda-host-only > output.txt

        # decide if error or warning fail
        if [[ -n $(grep "warning: " output.txt) ]] || [[ -n $(grep "error: " output.txt) ]]; then
            echo ""
            echo "You must pass the clang tidy checks before submitting a pull request"
            echo ""
            grep --color -E '^|warning: |error: ' output.txt
            if [[ -n $(grep "error: " output.txt) ]]; then
                exit_flag=true
                filesWithErrors=( "${filesWithErrors[@]}" $f )
                echo -e "\033[1;31m\xE2\x9C\x98 failed file $f\033[0m $1";
            else
                echo -e "\033[1;33m\xE2\x9C\x93 passed file $f with warnings\033[0m $1";
            fi
        else
            echo -e "\033[1;32m\xE2\x9C\x93 passed file $f\033[0m $1";
        fi
        rm output.txt
    else
        echo "$f not a C++ file or not in compilation database (compile_commands.json)"
    fi
done


if [ "$exit_flag" = true ]; then
    echo "Error in file(s):"
    for f in "${filesWithErrors[@]}"; do
        echo "$f"
    done
    exit -1
fi

echo "clang-tidy check passed"

exit 0
