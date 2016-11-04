var SbgnPDNode = require('./SbgnPDNode');

function SbgnProcessNode(gm, loc, size, vNode) {
  SbgnPDNode.call(this, gm, loc, size, vNode);
}

SbgnProcessNode.prototype = Object.create(SbgnPDNode.prototype);
for (var prop in SbgnPDNode) {
  SbgnProcessNode[prop] = SbgnPDNode[prop];
}

module.exports = SbgnProcessNode;
