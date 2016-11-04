var CoSEEdge = require('./CoSEEdge');

function SbgnPDEdge(source, target, vEdge) {
  CoSEEdge.call(this, source, target, vEdge);
}

CoSEEdge.prototype = Object.create(CoSEEdge.prototype);
for (var prop in CoSEEdge) {
  SbgnPDEdge[prop] = CoSEEdge[prop];
}

module.exports = SbgnPDEdge;
