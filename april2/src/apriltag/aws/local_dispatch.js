'use strict';

/* npm install aws-sdk */

let aws = require('aws-sdk');
aws.config.update({region: "us-east-1"});

let s3 = new aws.S3({ apiVersion: '2006-03-01' });
let lambda = new aws.Lambda;
let ebolson = require('./ebolson');
let fs = require('fs');

dispatch("apriltag-data",                    /* bucketName */
         [ ".JPG", ".jpg", ".PNG", ".png" ], /* keyTypes */
         "leaf_apriltag",                    /* lambda functionName */
         "-x 2 -t 2",                        /* arguments */
         "results.json",                     /* outputFile */
         -1,                                 /* maxCalls, -1 for all */
         1000                                 /* maxConcurrent */
        );

function dispatchlog(key, msg)
{
    let ts = (new Date().getTime()); // / 1000.0).toPrecision(3);

    console.log(ts+" "+ebolson.padRight(key, 80)+" : " + msg);
}

function dispatch(bucketName, keyTypes, functionName, functionArguments, outputFile,  maxCalls, maxConcurrent)
{
    let results = [];
    let startTime = new Date().getTime();

    function runOne(key, callback)
    {
        dispatchlog(key, "invoke");

        let payload = { "Bucket" : bucketName,
                        "Key" : key,
                        "Arguments" : functionArguments
                      };

        let invoke_params = { "FunctionName" : functionName,
                              "InvocationType" : "RequestResponse",
                              "LogType" : "Tail",
                              "Payload" : JSON.stringify(payload)
                            };

        lambda.invoke(invoke_params,
                      function(err, data) {

                          callback(err,
                                   {
                                       "Key": key,
                                       "Error" : err,
                                       "Data" : data
                                   }
                                  );
                      });
    }

    function runAll(keys)
    {
        let keys_retry = [];

        ebolson.map_parallel(runOne,
                             keys,
                             maxConcurrent,

                             function(key, result) {

                                 // on_each
                                 if (result.Error != null) {
                                     dispatchlog(key, "error "+result.Error);

                                     if (result.Error.statusCode == 429) {

                                         keys_retry.push(key);
                                         return;
                                     }
                                 }

                                 dispatchlog(key, "done");
                                 results.push(result);
                             },

                             function() {
                                 // on_finished
                                 keys = keys_retry;
                                 keys_retry = [];

                                 if (keys.length == 0) {
                                     let output = { "Results" : results,
                                                    "StartTime" : startTime,
                                                    "EndTime"   : new Date().getTime(),
                                                    "Bucket" : bucketName,
                                                    "KeyTypes" : keyTypes,
                                                    "FunctionName" : functionName,
                                                    "MaxCalls" : maxCalls,
                                                    "MaxConcurrent" : maxConcurrent,
                                                    "Arguments" : functionArguments};

                                     fs.writeFileSync(outputFile, JSON.stringify(output));
                                 }

                                 console.log("finished a batch, keys to retry: "+keys.length);

                                 runAll(keys);
                             });
    }

    ebolson.bucketGetAllKeys(bucketName, function(keys) {

        console.log("all keys: "+keys.length);

        keys = keys.filter(function(el) {

            if (el.includes("mosaic"))
                return false;

            for (var i = 0; i < keyTypes.length; i++) {

                if (el.endsWith(keyTypes[i]))
                    return true;
            }

            return false;
        });

        if (maxCalls >= 0 && keys.length > maxCalls) {
            keys = keys.splice(0, maxCalls);
        }

        keys = keys.sort();

        console.log("filtered keys: "+keys.length);

        runAll(keys);
    });

} // dispatch
