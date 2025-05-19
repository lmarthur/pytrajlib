module.exports = {
  theme: {
    extend: {
      fontFamily: {
        varela: ['var(--font-varela)', 'monospace'],
      },
    },
  },
  content: [
    './app/**/*.{js,ts,jsx,tsx}',
    './pages/**/*.{js,ts,jsx,tsx}',
    './components/**/*.{js,ts,jsx,tsx}',
  ],
};
