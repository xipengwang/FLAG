'use strict';

let aws = require('aws-sdk');
let s3 = new aws.S3({ apiVersion: '2006-03-01' });
let fs = require('fs');
let child_process = require('child_process');
let ebolson = require('./ebolson');

exports.handler = (event, context, finish) => {

    ebolson.waterfall([
        function(callback) {
            const params = {
                Bucket: event.Bucket,
                Key: event.Key,
            };

            var file = fs.createWriteStream("/tmp/test.jpg");
            var stream = s3.getObject(params).createReadStream().pipe(file);
            stream.on('finish', function() { callback(null); });

        },
        function(callback) {
            var cmd = "./apriltag_aws /tmp/test.jpg -j /tmp/test.json " + event.Arguments;
            console.log("cmd: "+cmd);
            child_process.exec(cmd, callback);
        },
        function(stdout, stderr, callback) {
            var result = JSON.parse(fs.readFileSync("/tmp/test.json"));
            callback(null, result); // no error!
        }
    ],
                      function(err, stdout) {
                          finish(err, stdout);
                      }
                     );
};
