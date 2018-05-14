'use strict';

let aws = require('aws-sdk');
let s3 = new aws.S3({ apiVersion: '2006-03-01' });

// func(key, callback): a function to be applied to 'key' that should
// call callback(err, result) upon completion.
//
// max: maximum number of functions to be invoked at once.
//
// finished(results): a function to be called when all work is
// done. Results is an array of { Key, Error, Result }

module.exports.map_parallel = map_parallel;

function map_parallel(func, datas, max, on_each, on_finished)
{
    var results = [ ];

    var ncalls = 0;      // how many calls have we begun?
    var nconcurrent = 0; // how many calls in progress?
    var ncallbacks = 0;  // how many callbacks have been called?

    var run_another = function() {
        while (nconcurrent < max && ncalls < datas.length) {
            nconcurrent++;

            var key = datas[ncalls];
            ncalls++;

            var callback = function(key) {
                return function(err, result) {

                    var result = { "Key" : key,
                                   "Error" : err,
                                   "Result" : result };

                    on_each(key, result);

                    nconcurrent--;

                    ncallbacks++;
                    if (ncallbacks == datas.length)
                        on_finished();

                    run_another();
                };
            }(key);

            func(key, callback);
        }
    }

    run_another();
};

////////////////////////////////////////////////////////////////////

// called with the Arguments of callback()
function waterfall_r(taskidx, tasks, finished, args)
{
    if (taskidx >= tasks.length || args[0] !== null) {
        finished.apply(null, args);
    } else {
        // copy all but first error argument to output.
        // Note node.js doesn't have Array.from.
        var callargs = [];
        for (var i = 1; i < args.length; i++)
    	    callargs.push(args[i]);

        callargs.push(function() {
    	    waterfall_r(taskidx+1, tasks, finished, arguments);
        });

        tasks[taskidx].apply(null, callargs);
    }
}

module.exports.waterfall = waterfall;

function waterfall(tasks, finished)
{
    waterfall_r(0, tasks, finished, [ null ]);
}

//////////////////////////////////////////////////////


// Reads keys from a bucket, calling 'callback(key)' as they are
// read. 'finish()' is called when all keys have been read.
function bucketGetAllKeys_r(bucket, marker, callback, finish)
{
    s3.listObjects({Bucket: bucket, Marker: marker }, function (err, data) {
        if (err) {
            console.log("err: "+err);
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

module.exports.bucketGetAllKeys = bucketGetAllKeys;

// Read all keys, calling 'callback(keys)' upon completion.
function bucketGetAllKeys(bucket, callback)
{
    let keys = [];

    bucketGetAllKeys_r(bucket, null,
                       function(key) {
                           keys.push(key);
                       },
                       function() {
                           callback(keys);
                       });
}

// return 'len' spaces. If len <= 0, returns empty string.
module.exports.spaces = spaces;
function spaces(len)
{
    if (len <= 0)
        return "";
    if (len == 1)
        return " ";

    let s = spaces(Math.floor(len/2));
    s = s + s;
    if (s.length < len)
        s = s + " ";
    return s;
}

// returns a string with additional spaces on the right until its
// length is 'len'. If 'str' is longer than len, the original string
// is returned.
module.exports.padRight = padRight;
function padRight(str, len)
{
    return str + spaces(len - str.length);
}

module.exports.padLeft = padLeft
function padLeft(str, len)
{
    return spaces(len - str.length) + str;
}

module.exports.formatNumber = formatNumber;
function formatNumber(v, width, prec)
{
    var s = v.toFixed(prec);
    if (width < 0)
        return padRight(s, width);
    return padLeft(s, width);
}

module.exports.atob = atob;
function atob(b64)
{
    return new Buffer(b64, 'base64').toString('binary')
}
