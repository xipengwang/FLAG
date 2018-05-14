# Process tests logfile and generate summary of test results
logfile=$1
fail_logfile=$(dirname $logfile)/tests_failed.log

tests_total=0
tests_passed=0
tests_failed=0

tests_passed=`grep "UTEST PASS" $logfile -c`
tests_failed=`grep "UTEST FAIL" $logfile -c`
tests_total=$((tests_failed+tests_passed))

grep "UTEST FAIL" $logfile > $fail_logfile

printf '=%.s' `seq 1 80`
echo
echo $tests_total "Tests" $tests_passed "Passed" $tests_failed "Failed"

if [ $tests_failed -ne 0 ]; then
    echo "See failed tests in "$fail_logfile
fi
