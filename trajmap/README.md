## Getting Started

Install nvm https://github.com/nvm-sh/nvm. Then run the following to install npm:
```bash
nvm install node
```

Change to the website directory and install dependencies:
```bash
cd trajmap
npm install
```

Then start the development server:
```bash
npm run dev
```

Open [http://localhost:3000](http://localhost:3000) with your browser to see the site.

## Changing C code
After modifying the Monte Carlo C code, recompile it with [Emscripten](https://emscripten.org/).

First, [install Emscripten](https://emscripten.org/docs/getting_started/downloads.html). Then run:

```bash
source scripts/emscript.sh
```

## Tools used
- The ReactJS framework [NextJS](https://nextjs.org/)
- [Nominatim](https://github.com/osm-search/Nominatim) for location searches with
OSM locations