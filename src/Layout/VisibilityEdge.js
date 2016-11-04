var CoSEEdge = require('./CoSEEdge');

function VisibilityEdge(source, target, vEdge) {
  CoSEEdge.call(this, source, target, vEdge);
}

VisibilityEdge.prototype = Object.create(CoSEEdge.prototype);
for (var prop in CoSEEdge) {
  VisibilityEdge[prop] = CoSEEdge[prop];
}

module.exports = VisibilityEdge;
