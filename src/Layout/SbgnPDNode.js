var CoSENode = require('./CoSENode');

function SbgnPDNode(gm, loc, size, vNode) {
  CoSENode.call(this, gm, loc, size, vNode);
}

SbgnPDNode.prototype = Object.create(CoSENode.prototype);
for (var prop in CoSENode) {
  SbgnPDNode[prop] = CoSENode[prop];
}

module.exports = SbgnPDNode;
