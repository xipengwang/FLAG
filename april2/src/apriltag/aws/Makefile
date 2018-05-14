.phony = run upload

all: leaf_apriltag

########################################################################
# LEAF NOP

leaf_nop.zip: leaf_nop.js
	zip -j $@ $^

leaf_nop: leaf_nop.zip
	-aws lambda delete-function --function-name leaf_nop
	aws lambda create-function --function-name leaf_nop --runtime nodejs4.3 --role arn:aws:iam::417553287511:role/lambda_s3_exec_role --handler leaf_nop.handler --memory-size 128 --timeout 20 --zip-file fileb://leaf_nop.zip

########################################################################
# LEAF APRILTAG

leaf_apriltag.zip: leaf_apriltag.js ebolson.js apriltag_aws
	zip -j $@ $^

leaf_apriltag: leaf_apriltag.zip
	-aws lambda delete-function --function-name leaf_apriltag
	aws lambda create-function --function-name leaf_apriltag --runtime nodejs4.3 --role arn:aws:iam::417553287511:role/lambda_s3_exec_role --handler leaf_apriltag.handler --memory-size 1536 --timeout 300 --zip-file fileb://leaf_apriltag.zip

########################################################################

dispatch.zip: dispatch.js ebolson.js
	zip -j $@ $^

dispatch:	dispatch.zip
	-aws lambda delete-function --function-name dispatch
	aws lambda create-function --function-name dispatch --runtime nodejs4.3 --role arn:aws:iam::417553287511:role/lambda_dispatch_role --handler dispatch.handler --memory-size 128 --timeout 300 --zip-file fileb://dispatch.zip
	aws lambda invoke --function-name dispatch --invocation-type RequestResponse --payload '{ "Bucket": "apriltag-data", "FunctionName": "leaf_apriltag", "MaxCalls": -1, "MaxConcurrent": 50, "KeyTypes": [ "JPG", "jpg", "PNG", "png"] }' --log-type Tail /tmp/dispatch.out | tee /tmp/dispatch.log
	cat /tmp/dispatch.log | perl -ne '/LogResult": "([^"]*)/ && print $$1' | base64 --decode

run:	upload
	aws lambda create-function --function-name dispatch --runtime nodejs --role lambda_s3_exec_role --handler dispatch --memory-size 512M --zip-file dispatch.zip
	aws lambda invoke --function-name s3test  --log-type Tail /tmp/aws.out | tee /tmp/aws.log
	cat /tmp/aws.log | perl -ne '/LogResult": "([^"]*)/ && print $$1' |base64 --decode

helloworld: helloworld.c
	gcc -o $@ $^ -static -Os


clean:
	rm -f hello.zip helloworld
