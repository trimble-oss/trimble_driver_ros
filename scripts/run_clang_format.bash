#!/bin/bash

# exit when any command fails
set -e

format_files () {
  local dir=$1
  shift  # shift all arguments to the left
  local excluded=("$@")

  pushd ${dir} > /dev/null

  exclude_cmds=$(printf "! -path %s " "${excluded[@]}")

  files=$(find . \
    -path './external' -prune -o \
    -type f \( -iname *.cpp -o -iname *.hpp -o -iname *.c -o -iname *.cc -o -iname *.h  \) \
    ${exclude_cmds} \
    -print)

  clang-format --style=file \
    -i \
    ${files}

  popd > /dev/null
}

git_hook () {
  # In this mode we assume the files to be formatted have been staged but not committed yet
  local excluded=("$@")
  pushd .

  cd "$(git rev-parse --show-toplevel)"
  mapfile -t cpp_files < <(git diff --cached --name-only --diff-filter=ACMRT | grep -E "\.(c|cpp|inl|h|hpp|cc|cxx|hxx)$")
  if [[ -n ${cpp_files} ]]; then
    echo "Running clang-format on the following file(s):"
    echo "--------------------------------------------"
    for file in ${cpp_files[@]} ; do
      echo "- ${file}"
    done
    echo

    # Expand all paths
    for i in ${!cpp_files[@]} ; do
      cpp_files[$i]="$(pwd)/${cpp_files[$i]}"
      file=${cpp_files[$i]}
    done

    # Expand paths of excluded files
    for i in ${!excluded[@]} ; do
      excluded[$i]=$(readlink -f ${excluded[$i]})
    done

    # Remove excluded files from files requirering formatting
    cpp_files_to_format=()
    for f in "${cpp_files[@]}"; do
      skip=
      for f_excluded in "${excluded[@]}"; do
        [[ $f == $f_excluded ]] && { skip=1; break; }
      done
      [[ -n $skip ]] || cpp_files_to_format+=("${f}")
    done

    for file_to_format in "${cpp_files_to_format[@]}"; do
      if [ -f "$file_to_format" ]; then
        clang-format -i -style=file "${file_to_format}"
        git add "${file_to_format}"
      fi
    done
  fi

  popd > /dev/null
}


excludes=(
    # Add paths to excluded files here
    "./trimble_driver_ros/include/trimble_driver/util/wise_enum/compact_optional.h" \
    "./trimble_driver_ros/include/trimble_driver/util/wise_enum/optional_common.h" \
    "./trimble_driver_ros/include/trimble_driver/util/wise_enum/optional.h" \
    "./trimble_driver_ros/include/trimble_driver/util/wise_enum/wise_enum_detail.h" \
    "./trimble_driver_ros/include/trimble_driver/util/wise_enum/wise_enum_generated.h" \
    "./trimble_driver_ros/include/trimble_driver/util/wise_enum/wise_enum.h"
    )


retval=1
if [[ $1 == "hook" ]]; then
  git_hook "${lks_excluded[@]}"
  exit 0;
else
  format_files "." "${excludes[@]}"

  changed_files=$(git status --ignore-submodules -- '*.cpp' '*.hpp' '*.c' '*.cc' '*.h' | grep "modified" | sed -n -e 's/modified: //p')
  if [[ -z $changed_files ]]; then
    echo "Style check passed."
    exit 0;
  fi

  if [[ $1 == "ci" ]]; then
    echo "Style check failed. Please run scripts/run_clang_format.bash to format the following files:"
    echo "$changed_files"
    retval=1
  else
    echo "Formatted files."
    retval=0
  fi
fi

exit $retval;
