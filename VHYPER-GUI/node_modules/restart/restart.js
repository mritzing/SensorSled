#! /usr/bin/env node

var exec = require('child_process').exec;
var fs = require('fs');
var path = require('path');

// Text editors often change multiple files. Need to throttle restarts
function debounce(fn, delay) {
  var timer = null;
  return function () {
    var context = this, args = arguments;
    clearTimeout(timer);
    timer = setTimeout(function () {
      fn.apply(context, args);
    }, delay);
  };
}

// parse args
var args = process.argv.reduce(function(acc, arg) {
  if (['--watch', '--exclude', '--exec'].some(function(opt) {
      return opt === arg;
  })) {
    acc.current = arg.substring(2);
    acc[acc.current] = [];
    return acc;
  }
  if (acc.current)
    acc[acc.current].push(arg);
  return acc;
}, {});

if (!args.exec || !args.watch) {
  console.log('Example usage:');
  console.log('restart --watch . --exclude node_modules public .git --exec node server.js');
  console.log('--watch : list of files and/or directories to watch');
  console.log('--exclude : list of files and/or directories to exclude from watching');
  console.log('--exec : Command starting a program. It will be killed and restarted when a watched file changes');
  return;
}

args.exclude = (args.exclude || []).map(path.normalize);

function restart() {
  console.log('(re)starting!');
  if (proc)
    proc.kill();
  proc = exec(args.exec.join(' '), function(err, stdout, stderr) {
    if (err) console.log(err);
    process.stdout.write(stdout);
    process.stderr.write(stderr);
  });
}

var proc;
restart();

function walk(name) {
  // TODO: Wildchar
  if (args.exclude.some(function(ignore) {
    return ignore === path.normalize(name);
  }))
    return;

  fs.stat(name, function(err, stat) {
    if (stat && stat.isDirectory()) {
      // fs.watch(name, debounce(restart, 200)); // Add to use fs.watch
      fs.readdir(name, function(err, files) {
        files.forEach(function(file) {
          walk(path.join(name, file));
        });
      });
    }
    if (stat && stat.isFile()) {
      fs.watchFile(name, restart); // Remove to use fs.watch
    }
  });
}

// while fs.watch should be more efficient it relies too much on the system.
// Had problems using it on shared folders in a virtual machine.
args.watch.forEach(function(path) {
  fs.stat(path, function(err, stat) {
    if (stat.isFile())
      fs.watchFile(path, restart);
    else
      walk(path);
  });
});

process.on('SIGINT', function() {
  proc.kill();
  process.exit();
});
