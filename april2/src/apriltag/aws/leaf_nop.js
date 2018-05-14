'use strict';

// params:
//   Bucket:        name of the bucket
//   Key:           name of the bucket's key

exports.handler = function(event, context, finish)
{
    finish(null, "bucket: "+event.Bucket+", key: "+event.Key);
}
