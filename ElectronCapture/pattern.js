var PatternLengthInBeats = 16;
var JugglerSize = 25;
var Jugglers = [
 { color: '#0000FF',
      ts: [0, 2, 4, 6, 8, 10, 12, 14, 16, ],
       x: [45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0, 45.0 ],
       y: [241.0, 241.0, 241.0, 241.0, 241.0, 241.0, 241.0, 241.0, 241.0 ],
      fa: [0.0, -0.4, 0.0, 0.4, 0.0, -0.4, 0.0, 0.4, 0.0 ] },
 { color: '#008000',
      ts: [0, 2, 4, 6, 8, 10, 12, 14, 16, ],
       x: [259.0, 259.0, 259.0, 259.0, 259.0, 259.0, 259.0, 259.0, 259.0 ],
       y: [337.0, 337.0, 337.0, 337.0, 337.0, 337.0, 337.0, 337.0, 337.0 ],
      fa: [-1.6, -0.4, 1.6, -2.7, -1.6, -0.4, 1.6, -2.7, -1.6 ] },
 { color: '#808000',
      ts: [0, 2, 4, 6, 8, 10, 12, 14, 16, ],
       x: [149.0, 139.0, 259.0, 379.0, 369.0, 377.0, 259.0, 141.0, 149.0 ],
       y: [243.0, 100.0, 48.0, 100.0, 243.0, 383.0, 436.0, 383.0, 243.0 ],
      fa: [-3.1, -3.9, 1.6, 0.8, -0.0, -0.8, -1.6, -2.3, -3.1 ] },
 { color: '#FF0000',
      ts: [0, 2, 4, 6, 8, 10, 12, 14, 16, ],
       x: [259.0, 259.0, 259.0, 259.0, 259.0, 259.0, 259.0, 259.0, 259.0 ],
       y: [147.0, 147.0, 147.0, 147.0, 147.0, 147.0, 147.0, 147.0, 147.0 ],
      fa: [1.6, 2.7, -1.6, 0.4, 1.6, 2.7, -1.6, 0.4, 1.6 ] },
 { color: '#FF00FF',
      ts: [0, 2, 4, 6, 8, 10, 12, 14, 16, ],
       x: [369.0, 377.0, 259.0, 141.0, 149.0, 139.0, 259.0, 379.0, 369.0 ],
       y: [243.0, 383.0, 436.0, 383.0, 243.0, 100.0, 48.0, 100.0, 243.0 ],
      fa: [-0.0, -0.8, -1.6, -2.3, -3.1, -3.9, 1.6, 0.8, -0.0 ] },
 { color: '#FF8000',
      ts: [0, 2, 4, 6, 8, 10, 12, 14, 16, ],
       x: [474.0, 474.0, 474.0, 474.0, 474.0, 474.0, 474.0, 474.0, 474.0 ],
       y: [241.0, 241.0, 241.0, 241.0, 241.0, 241.0, 241.0, 241.0, 241.0 ],
      fa: [3.1, 2.7, 3.1, -2.7, 3.1, 2.7, 3.1, -2.7, 3.1 ] },
];
var Passes = [
  { start: 0, end: 1.3, from: 0, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 0, end: 1.3, from: 1, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 0, end: 1.3, from: 2, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 0, end: 1.3, from: 3, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 0, end: 1.3, from: 4, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 0, end: 1.3, from: 5, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 2, end: 3.3, from: 0, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 2, end: 3.3, from: 1, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 2, end: 3.3, from: 2, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 2, end: 3.3, from: 3, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 2, end: 3.3, from: 4, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 2, end: 3.3, from: 5, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 4, end: 5.3, from: 0, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 4, end: 5.3, from: 1, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 4, end: 5.3, from: 2, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 4, end: 5.3, from: 3, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 4, end: 5.3, from: 4, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 4, end: 5.3, from: 5, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 6, end: 7.3, from: 0, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 6, end: 7.3, from: 1, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 6, end: 7.3, from: 2, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 6, end: 7.3, from: 3, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 6, end: 7.3, from: 4, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 6, end: 7.3, from: 5, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 8, end: 9.3, from: 0, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 8, end: 9.3, from: 1, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 8, end: 9.3, from: 2, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 8, end: 9.3, from: 3, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 8, end: 9.3, from: 4, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 8, end: 9.3, from: 5, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 10, end: 11.3, from: 0, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 10, end: 11.3, from: 1, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 10, end: 11.3, from: 2, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 10, end: 11.3, from: 3, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 10, end: 11.3, from: 4, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 10, end: 11.3, from: 5, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 12, end: 13.3, from: 0, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 12, end: 13.3, from: 1, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 12, end: 13.3, from: 2, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 12, end: 13.3, from: 3, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 12, end: 13.3, from: 4, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 12, end: 13.3, from: 5, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 14, end: 15.3, from: 0, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 14, end: 15.3, from: 1, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 14, end: 15.3, from: 2, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 14, end: 15.3, from: 3, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 14, end: 15.3, from: 4, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 14, end: 15.3, from: 5, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 16, end: 17.3, from: 0, fromhand: 'r', to: 2, tohand: 'l' },
  { start: 16, end: 17.3, from: 1, fromhand: 'r', to: 3, tohand: 'l' },
  { start: 16, end: 17.3, from: 2, fromhand: 'r', to: 0, tohand: 'l' },
  { start: 16, end: 17.3, from: 3, fromhand: 'r', to: 1, tohand: 'l' },
  { start: 16, end: 17.3, from: 4, fromhand: 'r', to: 5, tohand: 'l' },
  { start: 16, end: 17.3, from: 5, fromhand: 'r', to: 4, tohand: 'l' },
  { start: 1, end: 2.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 1, end: 2.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 1, end: 2.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 1, end: 2.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 1, end: 2.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 1, end: 2.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 3, end: 4.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 3, end: 4.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 3, end: 4.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 3, end: 4.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 3, end: 4.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 3, end: 4.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 5, end: 6.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 5, end: 6.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 5, end: 6.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 5, end: 6.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 5, end: 6.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 5, end: 6.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 7, end: 8.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 7, end: 8.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 7, end: 8.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 7, end: 8.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 7, end: 8.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 7, end: 8.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 9, end: 10.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 9, end: 10.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 9, end: 10.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 9, end: 10.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 9, end: 10.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 9, end: 10.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 11, end: 12.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 11, end: 12.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 11, end: 12.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 11, end: 12.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 11, end: 12.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 11, end: 12.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 13, end: 14.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 13, end: 14.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 13, end: 14.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 13, end: 14.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 13, end: 14.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 13, end: 14.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 15, end: 16.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 15, end: 16.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 15, end: 16.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 15, end: 16.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 15, end: 16.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 15, end: 16.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: 17, end: 18.3, from: 0, fromhand: 'l', to: 0, tohand: 'r' },
  { start: 17, end: 18.3, from: 1, fromhand: 'l', to: 1, tohand: 'r' },
  { start: 17, end: 18.3, from: 2, fromhand: 'l', to: 2, tohand: 'r' },
  { start: 17, end: 18.3, from: 3, fromhand: 'l', to: 3, tohand: 'r' },
  { start: 17, end: 18.3, from: 4, fromhand: 'l', to: 4, tohand: 'r' },
  { start: 17, end: 18.3, from: 5, fromhand: 'l', to: 5, tohand: 'r' },
  { start: -1.7, end: 0, who: 0, hand: 'r', held: true },
  { start: -1.7, end: 0, who: 1, hand: 'r', held: true },
  { start: -1.7, end: 0, who: 2, hand: 'r', held: true },
  { start: -1.7, end: 0, who: 3, hand: 'r', held: true },
  { start: -1.7, end: 0, who: 4, hand: 'r', held: true },
  { start: -1.7, end: 0, who: 5, hand: 'r', held: true },
  { start: -0.7, end: 1, who: 0, hand: 'l', held: true },
  { start: -0.7, end: 1, who: 1, hand: 'l', held: true },
  { start: -0.7, end: 1, who: 2, hand: 'l', held: true },
  { start: -0.7, end: 1, who: 3, hand: 'l', held: true },
  { start: -0.7, end: 1, who: 4, hand: 'l', held: true },
  { start: -0.7, end: 1, who: 5, hand: 'l', held: true },
  { start: 0.3, end: 2, who: 0, hand: 'r', held: true },
  { start: 0.3, end: 2, who: 1, hand: 'r', held: true },
  { start: 0.3, end: 2, who: 2, hand: 'r', held: true },
  { start: 0.3, end: 2, who: 3, hand: 'r', held: true },
  { start: 0.3, end: 2, who: 4, hand: 'r', held: true },
  { start: 0.3, end: 2, who: 5, hand: 'r', held: true },
  { start: 1.3, end: 3, who: 0, hand: 'l', held: true },
  { start: 1.3, end: 3, who: 1, hand: 'l', held: true },
  { start: 1.3, end: 3, who: 2, hand: 'l', held: true },
  { start: 1.3, end: 3, who: 3, hand: 'l', held: true },
  { start: 1.3, end: 3, who: 4, hand: 'l', held: true },
  { start: 1.3, end: 3, who: 5, hand: 'l', held: true },
  { start: 2.3, end: 4, who: 0, hand: 'r', held: true },
  { start: 2.3, end: 4, who: 1, hand: 'r', held: true },
  { start: 2.3, end: 4, who: 2, hand: 'r', held: true },
  { start: 2.3, end: 4, who: 3, hand: 'r', held: true },
  { start: 2.3, end: 4, who: 4, hand: 'r', held: true },
  { start: 2.3, end: 4, who: 5, hand: 'r', held: true },
  { start: 3.3, end: 5, who: 0, hand: 'l', held: true },
  { start: 3.3, end: 5, who: 1, hand: 'l', held: true },
  { start: 3.3, end: 5, who: 2, hand: 'l', held: true },
  { start: 3.3, end: 5, who: 3, hand: 'l', held: true },
  { start: 3.3, end: 5, who: 4, hand: 'l', held: true },
  { start: 3.3, end: 5, who: 5, hand: 'l', held: true },
  { start: 4.3, end: 6, who: 0, hand: 'r', held: true },
  { start: 4.3, end: 6, who: 1, hand: 'r', held: true },
  { start: 4.3, end: 6, who: 2, hand: 'r', held: true },
  { start: 4.3, end: 6, who: 3, hand: 'r', held: true },
  { start: 4.3, end: 6, who: 4, hand: 'r', held: true },
  { start: 4.3, end: 6, who: 5, hand: 'r', held: true },
  { start: 5.3, end: 7, who: 0, hand: 'l', held: true },
  { start: 5.3, end: 7, who: 1, hand: 'l', held: true },
  { start: 5.3, end: 7, who: 2, hand: 'l', held: true },
  { start: 5.3, end: 7, who: 3, hand: 'l', held: true },
  { start: 5.3, end: 7, who: 4, hand: 'l', held: true },
  { start: 5.3, end: 7, who: 5, hand: 'l', held: true },
  { start: 6.3, end: 8, who: 0, hand: 'r', held: true },
  { start: 6.3, end: 8, who: 1, hand: 'r', held: true },
  { start: 6.3, end: 8, who: 2, hand: 'r', held: true },
  { start: 6.3, end: 8, who: 3, hand: 'r', held: true },
  { start: 6.3, end: 8, who: 4, hand: 'r', held: true },
  { start: 6.3, end: 8, who: 5, hand: 'r', held: true },
  { start: 7.3, end: 9, who: 0, hand: 'l', held: true },
  { start: 7.3, end: 9, who: 1, hand: 'l', held: true },
  { start: 7.3, end: 9, who: 2, hand: 'l', held: true },
  { start: 7.3, end: 9, who: 3, hand: 'l', held: true },
  { start: 7.3, end: 9, who: 4, hand: 'l', held: true },
  { start: 7.3, end: 9, who: 5, hand: 'l', held: true },
  { start: 8.3, end: 10, who: 0, hand: 'r', held: true },
  { start: 8.3, end: 10, who: 1, hand: 'r', held: true },
  { start: 8.3, end: 10, who: 2, hand: 'r', held: true },
  { start: 8.3, end: 10, who: 3, hand: 'r', held: true },
  { start: 8.3, end: 10, who: 4, hand: 'r', held: true },
  { start: 8.3, end: 10, who: 5, hand: 'r', held: true },
  { start: 9.3, end: 11, who: 0, hand: 'l', held: true },
  { start: 9.3, end: 11, who: 1, hand: 'l', held: true },
  { start: 9.3, end: 11, who: 2, hand: 'l', held: true },
  { start: 9.3, end: 11, who: 3, hand: 'l', held: true },
  { start: 9.3, end: 11, who: 4, hand: 'l', held: true },
  { start: 9.3, end: 11, who: 5, hand: 'l', held: true },
  { start: 10.3, end: 12, who: 0, hand: 'r', held: true },
  { start: 10.3, end: 12, who: 1, hand: 'r', held: true },
  { start: 10.3, end: 12, who: 2, hand: 'r', held: true },
  { start: 10.3, end: 12, who: 3, hand: 'r', held: true },
  { start: 10.3, end: 12, who: 4, hand: 'r', held: true },
  { start: 10.3, end: 12, who: 5, hand: 'r', held: true },
  { start: 11.3, end: 13, who: 0, hand: 'l', held: true },
  { start: 11.3, end: 13, who: 1, hand: 'l', held: true },
  { start: 11.3, end: 13, who: 2, hand: 'l', held: true },
  { start: 11.3, end: 13, who: 3, hand: 'l', held: true },
  { start: 11.3, end: 13, who: 4, hand: 'l', held: true },
  { start: 11.3, end: 13, who: 5, hand: 'l', held: true },
  { start: 12.3, end: 14, who: 0, hand: 'r', held: true },
  { start: 12.3, end: 14, who: 1, hand: 'r', held: true },
  { start: 12.3, end: 14, who: 2, hand: 'r', held: true },
  { start: 12.3, end: 14, who: 3, hand: 'r', held: true },
  { start: 12.3, end: 14, who: 4, hand: 'r', held: true },
  { start: 12.3, end: 14, who: 5, hand: 'r', held: true },
  { start: 13.3, end: 15, who: 0, hand: 'l', held: true },
  { start: 13.3, end: 15, who: 1, hand: 'l', held: true },
  { start: 13.3, end: 15, who: 2, hand: 'l', held: true },
  { start: 13.3, end: 15, who: 3, hand: 'l', held: true },
  { start: 13.3, end: 15, who: 4, hand: 'l', held: true },
  { start: 13.3, end: 15, who: 5, hand: 'l', held: true },
  { start: 14.3, end: 16, who: 0, hand: 'r', held: true },
  { start: 14.3, end: 16, who: 1, hand: 'r', held: true },
  { start: 14.3, end: 16, who: 2, hand: 'r', held: true },
  { start: 14.3, end: 16, who: 3, hand: 'r', held: true },
  { start: 14.3, end: 16, who: 4, hand: 'r', held: true },
  { start: 14.3, end: 16, who: 5, hand: 'r', held: true },
  { start: 15.3, end: 17, who: 0, hand: 'l', held: true },
  { start: 15.3, end: 17, who: 1, hand: 'l', held: true },
  { start: 15.3, end: 17, who: 2, hand: 'l', held: true },
  { start: 15.3, end: 17, who: 3, hand: 'l', held: true },
  { start: 15.3, end: 17, who: 4, hand: 'l', held: true },
  { start: 15.3, end: 17, who: 5, hand: 'l', held: true },
  { start: 16.3, end: 18, who: 0, hand: 'r', held: true },
  { start: 16.3, end: 18, who: 1, hand: 'r', held: true },
  { start: 16.3, end: 18, who: 2, hand: 'r', held: true },
  { start: 16.3, end: 18, who: 3, hand: 'r', held: true },
  { start: 16.3, end: 18, who: 4, hand: 'r', held: true },
  { start: 16.3, end: 18, who: 5, hand: 'r', held: true },
  { start: 17.3, end: 19, who: 0, hand: 'l', held: true },
  { start: 17.3, end: 19, who: 1, hand: 'l', held: true },
  { start: 17.3, end: 19, who: 2, hand: 'l', held: true },
  { start: 17.3, end: 19, who: 3, hand: 'l', held: true },
  { start: 17.3, end: 19, who: 4, hand: 'l', held: true },
  { start: 17.3, end: 19, who: 5, hand: 'l', held: true },
];
BPM=100;
FPS=20;