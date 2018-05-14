'use strict';

let aws = require('aws-sdk');
let s3 = new aws.S3({ apiVersion: '2006-03-01' });
let lambda = new aws.Lambda;
let ebolson = require('./ebolson');

// Reads keys from a bucket, calling 'callback(key)' as they are
// read. 'finish()' is called when all keys have been read.
function bucketGetAllKeys_r(bucket, marker, callback, finish)
{
    s3.listObjects({Bucket: bucket, Marker: marker }, function (err, data) {
        if (err) {
            console.log(err);
        } else {
            for (var idx = 0; idx < data.Contents.length; idx++)
                callback(data.Contents[idx].Key);
            if (data.IsTruncated)
                listing(data.NextMarker, callback);
            else
                finish();
        }
    });
}

// Read all keys, calling 'callback(keys)' upon completion.
function bucketGetAllKeys(bucket, callback)
{
    var keys = [];

    bucketGetAllKeys_r(bucket, null,
                       function(key) {
                           keys.push(key);
                       },
                       function() {
                           callback(keys);
                       });
}

// params:
//   Bucket:        name of the bucket
//   FunctionName:  name of the lambda function
//   MaxCalls:      maximum number of objects in bucket to process (-1 for all)
//   MaxConcurrent: maximum number of calls to make in parallel
//   KeyTypes:      an array of file name suffixes that will be processed.
//                  e.g. [ "JPG", "jpg" ]. Use [ "" ] to process everything.
exports.handler = function(event, context, finish)
{
    function dowork(key, callback)
    {
        var payload = { "Bucket" : event.Bucket,
                        "Key" : key
                      };

        var invoke_params = { "FunctionName" : event.FunctionName,
                              "InvocationType" : "RequestResponse",
                              "LogType" : "Tail",
                              "Payload" : JSON.stringify(payload)
                            };

        lambda.invoke(invoke_params,
                      function(err, data) {
                          if (err != null && err.statusCode == 429) {
                              // TooManyRequestsException

                          }

                          callback(err,
                                   {
                                       "Key": key,
                                       "Error" : err,
                                       "Data" : data
                                   }
                                  );
                      });

//        callback(null, "result: "+key);
    }

    ebolson.waterfall(
        [
            function(callback) {
                bucketGetAllKeys(event.Bucket,
                                 function (keys) {
                                     callback(null, keys);
                                 });
            },

            function(keys, callback) {

                keys = keys.filter(function(el) {
                    for (var i = 0; i < event.KeyTypes.length; i++)
                        if (el.endsWith(event.KeyTypes[i]))
                            return true;
                    return false;
                });

                if (event.MaxCalls >= 0 && keys.length > event.MaxCalls) {
                    keys = keys.splice(0, event.MaxCalls);
                }

                ebolson.map_parallel(dowork, keys, event.MaxConcurrent,
                                     function(results) {
                                         callback(null, results);
                                     });
            },
        ],

        function (err, results) {
            finish(null, results);
        });

};
