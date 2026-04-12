#!/bin/bash --posix
#
# A hook script to verify what is about to be committed.
# Called by git-commit with no arguments.  The hook should
# exit with non-zero status after issuing an appropriate message if
# it wants to stop the commit.
#

if `git rev-parse --verify HEAD > /dev/null 2>&1`
then
        against=HEAD
else
        # Initial commit: diff against an empty tree object
        against=4b825dc642cb6eb9a060e54bf8d69288fbee4904
fi

BAD_FILE=0

# Find our changed files - only check A(dded), C(opied), M(odified), R(enamed)
list=`git diff-index --cached --name-only --diff-filter=ACMR $against`

function rejectbadpermissions {
    for item in $list
    do
        # perm is set via ls-files -s (staged), which reports the file
        # details like this:
        #    100755 f46832a3c69466bdf976557d20f071fb36b413c6 0       file.txt
        #
        # We pull out chars 4-6, which are the file permissions
        #
        # Note that we can't just 'stat --format %a ${item} b/c the current
        # permissions may not be what's staged in the index to be committed
        perm=`git ls-files -s ${item} | awk '{ print substr($0, 4, 3) }'`
        type=`file $item`
        if [[ ($type == *ASCII*) && ($type != *shell*) ]]; then
           if [ $perm != "644" ]; then
              echo "File $item is rejected due to incorrect permissions ($perm).  Text files should be 644"
              BAD_FILE=1
           fi
        fi
    done
}

# Reject commits containing incorrect usage of TI Trademarks such as:
# "SimpleLink", "LaunchPad", and "BoosterPack".
function rejectbadtrademarks {

    # Regular expression contain valid and invalid trademark usage.
    trademark_correct='((BoosterPack)|(LaunchPad)|(SimpleLink))'
    trademark_incorrect='((booster( ?)pack)|(launch( ?)pad)|(simple( ?)link))'

    bad_trademark=0

    for item in $list
    do
        type=`file $item`
        # file.1: ASCII text
        if [[ $type == *ASCII* ]]; then

            #grep This file for invalid TI trademarks
            count=`git grep --cached -i -E -h "(^| )$trademark_incorrect" $item | grep -P -v "$trademark_correct" | wc -w`

            if [[ $count -ne 0 ]]; then
                echo
                echo "File $item is rejected due to incorrect TI trademark usage."
                echo "==========================="
                grep -P -i -n -o --color=always "(^|(?<= ))$trademark_incorrect" $item | grep -P -v "$trademark_correct"
                echo "==========================="
                bad_trademark=1
                BAD_FILE=1
            fi
        fi
    done

    if [[ bad_trademark -eq 1 ]]; then
        echo
        echo "Proper trademark usage: SimpleLink, LaunchPad, BoosterPack"
        echo "If you know what you are doing you can disable this TISB"
        echo "trademark check using:"
        echo
        echo "  git config tisb.notrademarkcheck true"
        echo
    fi
}

# Function to support the 'rejectbadincludeorder' function.
# Contains an awk program to print all lines between 2 patterns.
# The patterns are `extern "C" {` and `#ifdef __cplusplus \n}`
function awk_IncludeCmd {
    gawk '/^extern\s*"C"\s*/ {
        if (/{/ || getline == /^{/) {
            while (getline) {
                if (/^#ifdef\s*__cplusplus/ && getline == /^}/) { break }
                print;
            }
        }
    }'
}

# Reject commits containing files which have #include directives
# inside an extern "C" block
function rejectbadincludeorder {

    bad_include=0

    for item in $list
    do
        type=`file $item`
        # file.1: ASCII text and C/C++ header or source
        if [[ ($type == *ASCII*) && ($item =~ \.(c|h|cpp)$) ]]; then

            if [[ `git show :$item | awk_IncludeCmd | grep -P --color='auto' "^#include .*"` && $? -eq 0 ]]; then
                echo
                echo "File $item is rejected due to potential"
                echo "incorrect #include usage in an extern \"C\" block."
                echo "==========================="
                git show :$item | awk_IncludeCmd | grep -P --color='auto' "^#include .*"
                echo "==========================="
                bad_include=1
                BAD_FILE=1
            fi
        fi
    done

    if [[ bad_include -eq 1 ]]; then
        echo
        echo "All includes shall be placed before the extern \"C\" block."
        echo "If you know what you are doing you can use \`git commit --no-verify\`"
        echo "or disable this TISB include order check using:"
        echo
        echo "  git config tisb.noincludeordercheck true"
        echo
    fi
}

function rejectdosfiles {

    for item in $list
    do
        type=`file $item`
        # file.1: ASCII text, with CRLF line terminators
        if [[ $type == *CRLF* ]]; then
           echo "File $item is rejected due to dos format"
           BAD_FILE=1
        fi
    done

}

function rejectunicodetextfiles {

    for item in $list
    do
        type=`file $item`

        # file.1:  UTF-8 Unicode text
        if [[ $type == *ASCII* || $type == *Unicode* ]]; then
            # Since $type is the file type in the file system, we must ensure
            # files staged do not have Unicode characters. This prevents
            # us from flagging Unicode characters that are not staged.
            if [[ `git show :$item | grep -Pho "[^\x00-\x7F]"` && $? -eq 0 ]]; then
                echo
                echo "File $item is rejected due to Unicode characters"
                echo "==========================="
                git show :$item | grep -Phn -m5 --color='auto' "[^\x00-\x7F]"
                echo "==========================="
                BAD_FILE=1
                bad_unicode=1
            fi
        fi
    done

    if [[ $bad_unicode -eq 1 ]]; then
        echo
        echo "Note: A Byte Order Mark (BOM) character (0xEF,0xBB,0xBF) may be"
        echo "the first bytes of the file (text stream), indicating a UTF-8 BOM encoded file."
        echo
        echo "If you know what you are doing you can disable this TISB"
        echo "Unicode check using:"
        echo
        echo "  git config tisb.nounicodecheck true"
        echo
    fi
}

function rejectbadcopyright {

    for item in $list
    do
        type=`file $item`

        # file.1: ASCII text
        if [[ $type == *ASCII* ]]; then
            curyear=`date +%Y`

            # Number of lines with 'Copyright' in them
            copyrightlines=`git grep --cached Copyright $item | wc -l`

            # Number of lines with the 'correct' copyright in them"
            correctlines=`git grep --cached Copyright $item | grep "Texas Instruments Incorporated" | grep $curyear | wc -l`

            if [[ $copyrightlines -gt $correctlines ]]; then

                echo "Error: File $item contains incorrect copyright (${correctlines} correct/${copyrightlines} total)"
                echo
                echo "  See http://wiki.sanb.design.ti.com/twiki/bin/view/Process/CopyrightBestPractices"
                echo
                echo "If you know what you are doing you can disable this TISB"
                echo "copyright check using:"
                echo
                echo "  git config tisb.nocopyrightcheck true"
                echo

                BAD_FILE=1
            fi
        fi
    done

}

function rejectbadwhitespace {
    for item in $list
    do
        type=`file $item`
        skip_tab_check=0

        # file.1: ASCII text
        if [[ $type == *ASCII* ]]; then
            if [[ $item == [mM]akefile ]] || [[ $item == [mM]akeunix ]] ||
               [[ $item == *.mak ]] || [[ $item == *.mk ]]; then
                whitespace_check="blank-at-eol,blank-at-eof,space-before-tab"
                skip_tab_check=1
            else
                whitespace_check="blank-at-eol,blank-at-eof,space-before-tab,tab-in-indent,tabwidth=4"
            fi

            if [[ `git -c core.whitespace=${whitespace_check} diff --cached --check $item` ]]; then
                echo "Error: File $item contains undesirable whitespace"
                echo
                echo "==========================="
                git -c core.whitespace=${whitespace_check} diff --cached --check $item
                echo "==========================="
                echo

                BAD_FILE=1
                bad_whitespace=1
            elif [[ $skip_tab_check -eq 0 ]]; then
                if git diff --cached $against -- $item | egrep '^\+' | egrep '	'>/dev/null
                then
                    echo "Error: File $item contains undesirable tabs"
                    echo
                    echo "==========================="
                    git diff --cached $against -- $item | egrep '^\+' | egrep '	'
                    echo "==========================="
                    echo

                    BAD_FILE=1
                    bad_whitespace=1
                fi
            fi
        fi
    done

    if [[ $bad_whitespace -eq 1 ]]; then
        echo
        echo "If you know what you are doing you can disable this TISB"
        echo "whitespace check using:"
        echo
        echo "  git config tisb.nowhitespacecheck true"
        echo
    fi

}

rejectbadpermissions
rejectdosfiles

# Conditionally check trademarks - default is on, but users can disable with
#     git config tisb.notrademarkcheck true
if [[ $(git config tisb.notrademarkcheck) == "true" ]]; then
    echo "Remark: tisb.notrademarkcheck set to true, skipping"
else
    rejectbadtrademarks
fi

# Conditionally check include order - default is on, but users can disable with
#     git config tisb.noincludeordercheck true
if [[ $(git config tisb.noincludeordercheck) == "true" ]]; then
    echo "Remark: tisb.noincludeordercheck set to true, skipping"
else
    rejectbadincludeorder
fi

# Conditionally check copyrights - default is on, but users can disable with
#     git config tisb.nocopyrightcheck true
if [[ $(git config tisb.nocopyrightcheck) == "true" ]]; then
    echo "Remark: tisb.nocopyrightcheck set to true, skipping"
else
    rejectbadcopyright
fi

# Conditionally check whitespace - default is on, but users can disable with
#     git config tisb.nowhitespacecheck true
if [[ $(git config tisb.nowhitespacecheck) == "true" ]]; then
    echo "Remark: tisb.nowhitespacecheck set to true, skipping"
else
    rejectbadwhitespace
fi

# Conditionally check Unicode - default is on, but users can disable with
#     git config tisb.nounicodecheck true
if [[ $(git config tisb.nounicodecheck) == "true" ]]; then
    echo "Remark: tisb.nounicodecheck set to true, skipping"
else
    rejectunicodetextfiles
fi

exit $BAD_FILE
