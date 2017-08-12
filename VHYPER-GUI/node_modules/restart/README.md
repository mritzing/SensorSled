# restart
A super simple watch librarty for restarting processes. 100 lines and no dependencies.
## Install
Global:

`npm install -g restart`

Local:

`npm install restart`

## Usage

`restart --watch server.js --exec node server.js`

`restart --watch shared --exclude shared/.git --exec browserify shared/main.js -o public/bundle.js`

## API

--watch: list of files and/or directories to watch

--exclude: list of files and/or directories to exclude from watching

--exec (re)start process with command and following arguments when a watched file changes

## TODO

* Wildchar notation
* Figure out when it's safe to use fs.watch
