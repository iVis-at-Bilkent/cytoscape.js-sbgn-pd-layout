var CoSELayout = require('./CoSELayout');

function SbgnPDLayout() {
  CoSELayout.call(this);
}

SbgnPDLayout.prototype = Object.create(CoSELayout.prototype);

for (var prop in CoSELayout) {
  SbgnPDLayout[prop] = CoSELayout[prop];
}

module.exports = SbgnPDLayout;
