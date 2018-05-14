'use strict';

var fs = require('fs');
var ebolson = require('./ebolson');

var path = process.argv[2];

console.log("loading "+path);

var parsed = JSON.parse(fs.readFileSync(path));
//console.log(unjsoned);
var raw = parsed.Results;

var results = [ ];

var totalBilledMS = 0;
var totalActualMS = 0;
var totalGBtime = 0;
var maxMemoryUsed = 0;
var memorySize = 0;
var totalInvocations = 0;

for (var rawidx = 0; rawidx < raw.length; rawidx++) {
    let s = ebolson.atob(raw[rawidx].Result.Data.LogResult);
//    console.log(s);

    let billedMS = 0;
    let actualMS = 0;
    let thisMemorySize = 0; // how much we requested
    let memoryUsed = 0; // how much actually used

    if (true) {
        let matches = s.match(/Billed Duration: ([0-9]+)/);

        if (matches != null)
            billedMS = parseInt(matches[1]);
    }

    if (true) {
        let matches = s.match(/Duration: ([0-9\.]+)/);

        if (matches != null)
            actualMS = parseInt(matches[1]);
    }

    if (true) {
        let matches = s.match(/Memory Size: ([0-9]+)/);

        if (matches != null) {
            thisMemorySize = parseInt(matches[1]);
            if (memorySize != 0 && memorySize != thisMemorySize) {
                console.log("change in memory size! "+thisMemorySize+" "+memorySize);
            }
            memorySize = thisMemorySize;
        }
    }

    if (true) {
        let matches = s.match(/Max Memory Used: ([0-9]+)/);

        if (matches != null)
            memoryUsed = parseInt(matches[1]);
    }

    if (memoryUsed > maxMemoryUsed)
        maxMemoryUsed = memoryUsed;

    totalBilledMS += billedMS;
    totalActualMS += actualMS;
    totalGBtime += billedMS * memorySize / 1024.0 / 1000.0;
    totalInvocations ++;
}

console.log("----------------- Dispatch Statistics --------------------");
console.log("Total billed duration : " + (totalBilledMS / 1000).toFixed(1) + " s");
console.log("Total actual duration : " + (totalActualMS / 1000).toFixed(1) + " s");
console.log("Avg duration per key :  " + (totalActualMS / totalInvocations).toFixed(1) + " ms");
console.log("Total GB*s:             " + totalGBtime.toFixed(2) + " GB*s");
console.log("Memory used:            " + maxMemoryUsed + " MiB / " + memorySize + " MiB");
console.log("Total cost:             $" + ( 0.00001667 * totalGBtime).toFixed(4));
console.log("Wallclock time:         " + ((parsed.EndTime - parsed.StartTime) / 1000).toFixed(3) + " s");
console.log("Total invocations:      " + totalInvocations);
console.log("Arguments:              " + parsed.Arguments);
console.log(parsed.Arguments);
console.log("----------------- Dispatch Statistics --------------------");
console.log("\n");

for (var rawidx = 0; rawidx < raw.length; rawidx++) {

    if (raw[rawidx].Error != null) {
        console.log("ERROR "+raw[rawidx].Error.statusCode+" : "+raw[rawidx].Error.code);
        continue;
    }

    if (raw[rawidx].Result.Data == null) {
        console.log("ERROR: missing data for "+raw[rawidx].Key);
        continue;
    }

    results.push(JSON.parse(raw[rawidx].Result.Data.Payload));
}

var ndegrade = results[0].length;

for (var degradeidx = 0; degradeidx < ndegrade; degradeidx++) {
    var score = 0;

    for (var resultidx = 0; resultidx < results.length; resultidx++) {
        var result = results[resultidx];

        if (result[degradeidx].degrade != results[0][degradeidx].degrade)
            console.log("XXX");

        var detections = result[degradeidx].detections;

        for (var detidx = 0; detidx < detections.length; detidx++)
            score += 1.0 / (1 + detections[detidx].hamming);
    }

    console.log("degrade " + ebolson.formatNumber(results[0][degradeidx].degrade, 4, 2)+
                " score " + ebolson.formatNumber(score,10,3));
}
