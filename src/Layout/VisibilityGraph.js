var LGraph = require('./LGraph');

function VisibilityGraph(parent, graphMgr, vGraph) {
  LGraph.call(this, parent, graphMgr, vGraph);
}

VisibilityGraph.prototype = Object.create(LGraph.prototype);
for (var prop in LGraph) {
  VisibilityGraph[prop] = LGraph[prop];
}

module.exports = VisibilityGraph;
