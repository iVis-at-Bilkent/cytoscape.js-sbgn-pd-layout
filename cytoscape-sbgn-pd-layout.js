(function(f){if(typeof exports==="object"&&typeof module!=="undefined"){module.exports=f()}else if(typeof define==="function"&&define.amd){define([],f)}else{var g;if(typeof window!=="undefined"){g=window}else if(typeof global!=="undefined"){g=global}else if(typeof self!=="undefined"){g=self}else{g=this}g.cytoscapeSbgnPdLayout = f()}})(function(){var define,module,exports;return (function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(_dereq_,module,exports){
var FDLayoutConstants = _dereq_('./FDLayoutConstants');

function CoSEConstants() {
}

//CoSEConstants inherits static props in FDLayoutConstants
for (var prop in FDLayoutConstants) {
  CoSEConstants[prop] = FDLayoutConstants[prop];
}

CoSEConstants.DEFAULT_USE_MULTI_LEVEL_SCALING = false;
CoSEConstants.DEFAULT_RADIAL_SEPARATION = FDLayoutConstants.DEFAULT_EDGE_LENGTH;
CoSEConstants.DEFAULT_COMPONENT_SEPERATION = 60;

module.exports = CoSEConstants;

},{"./FDLayoutConstants":10}],2:[function(_dereq_,module,exports){
var FDLayoutEdge = _dereq_('./FDLayoutEdge');

function CoSEEdge(source, target, vEdge) {
  FDLayoutEdge.call(this, source, target, vEdge);
}

CoSEEdge.prototype = Object.create(FDLayoutEdge.prototype);
for (var prop in FDLayoutEdge) {
  CoSEEdge[prop] = FDLayoutEdge[prop];
}

module.exports = CoSEEdge;

},{"./FDLayoutEdge":11}],3:[function(_dereq_,module,exports){
var LGraph = _dereq_('./LGraph');

function CoSEGraph(parent, graphMgr, vGraph) {
  LGraph.call(this, parent, graphMgr, vGraph);
}

CoSEGraph.prototype = Object.create(LGraph.prototype);
for (var prop in LGraph) {
  CoSEGraph[prop] = LGraph[prop];
}

module.exports = CoSEGraph;

},{"./LGraph":19}],4:[function(_dereq_,module,exports){
var LGraphManager = _dereq_('./LGraphManager');

function CoSEGraphManager(layout) {
  LGraphManager.call(this, layout);
}

CoSEGraphManager.prototype = Object.create(LGraphManager.prototype);
for (var prop in LGraphManager) {
  CoSEGraphManager[prop] = LGraphManager[prop];
}

module.exports = CoSEGraphManager;

},{"./LGraphManager":20}],5:[function(_dereq_,module,exports){
var FDLayout = _dereq_('./FDLayout');
var CoSEGraphManager = _dereq_('./CoSEGraphManager');
var CoSEGraph = _dereq_('./CoSEGraph');
var CoSENode = _dereq_('./CoSENode');
var CoSEEdge = _dereq_('./CoSEEdge');

function CoSELayout() {
  FDLayout.call(this);
}

CoSELayout.prototype = Object.create(FDLayout.prototype);

for (var prop in FDLayout) {
  CoSELayout[prop] = FDLayout[prop];
}

CoSELayout.prototype.newGraphManager = function () {
  var gm = new CoSEGraphManager(this);
  this.graphManager = gm;
  return gm;
};

CoSELayout.prototype.newGraph = function (vGraph) {
  return new CoSEGraph(null, this.graphManager, vGraph);
};

CoSELayout.prototype.newNode = function (vNode) {
  return new CoSENode(this.graphManager, vNode);
};

CoSELayout.prototype.newEdge = function (vEdge) {
  return new CoSEEdge(null, null, vEdge);
};

CoSELayout.prototype.initParameters = function () {
  FDLayout.prototype.initParameters.call(this, arguments);
  if (!this.isSubLayout) {
    if (CoSEConstants.DEFAULT_EDGE_LENGTH < 10)
    {
      this.idealEdgeLength = 10;
    }
    else
    {
      this.idealEdgeLength = CoSEConstants.DEFAULT_EDGE_LENGTH;
    }

    this.useSmartIdealEdgeLengthCalculation =
            CoSEConstants.DEFAULT_USE_SMART_IDEAL_EDGE_LENGTH_CALCULATION;
    this.springConstant =
            FDLayoutConstants.DEFAULT_SPRING_STRENGTH;
    this.repulsionConstant =
            FDLayoutConstants.DEFAULT_REPULSION_STRENGTH;
    this.gravityConstant =
            FDLayoutConstants.DEFAULT_GRAVITY_STRENGTH;
    this.compoundGravityConstant =
            FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_STRENGTH;
    this.gravityRangeFactor =
            FDLayoutConstants.DEFAULT_GRAVITY_RANGE_FACTOR;
    this.compoundGravityRangeFactor =
            FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_RANGE_FACTOR;
  }
};

CoSELayout.prototype.layout = function () {
  var createBendsAsNeeded = LayoutConstants.DEFAULT_CREATE_BENDS_AS_NEEDED;
  if (createBendsAsNeeded)
  {
    this.createBendpoints();
    this.graphManager.resetAllEdges();
  }

  this.level = 0;
  return this.classicLayout();
};

CoSELayout.prototype.classicLayout = function () {
  this.calculateNodesToApplyGravitationTo();
  this.graphManager.calcLowestCommonAncestors();
  this.graphManager.calcInclusionTreeDepths();
  this.graphManager.getRoot().calcEstimatedSize();
  this.calcIdealEdgeLengths();
  if (!this.incremental)
  {
    var forest = this.getFlatForest();

    // The graph associated with this layout is flat and a forest
    if (forest.length > 0)

    {
      this.positionNodesRadially(forest);
    }
    // The graph associated with this layout is not flat or a forest
    else
    {
      this.positionNodesRandomly();
    }
  }

  this.initSpringEmbedder();
  this.runSpringEmbedder();

  console.log("Classic CoSE layout finished after " +
          this.totalIterations + " iterations");

  return true;
};

CoSELayout.prototype.runSpringEmbedder = function () {
  var lastFrame = new Date().getTime();
  var initialAnimationPeriod = 25;
  var animationPeriod = initialAnimationPeriod;
  do
  {
    this.totalIterations++;

    if (this.totalIterations % FDLayoutConstants.CONVERGENCE_CHECK_PERIOD == 0)
    {
      if (this.isConverged())
      {
        break;
      }

      this.coolingFactor = this.initialCoolingFactor *
              ((this.maxIterations - this.totalIterations) / this.maxIterations);
      animationPeriod = Math.ceil(initialAnimationPeriod * Math.sqrt(this.coolingFactor));

    }
    this.totalDisplacement = 0;
    this.graphManager.updateBounds();
    this.calcSpringForces();
    this.calcRepulsionForces();
    this.calcGravitationalForces();
    this.moveNodes();
    this.animate();
    if (FDLayoutConstants.ANIMATE === 'during' && this.totalIterations % animationPeriod == 0) {
      for (var i = 0; i < 1e7; i++) {
        if ((new Date().getTime() - lastFrame) > 25) {
          break;
        }
      }
      lastFrame = new Date().getTime();
      var allNodes = this.graphManager.getAllNodes();
      var pData = {};
      for (var i = 0; i < allNodes.length; i++) {
        var rect = allNodes[i].rect;
        var id = allNodes[i].id;
        pData[id] = {
          id: id,
          x: rect.getCenterX(),
          y: rect.getCenterY(),
          w: rect.width,
          h: rect.height
        };
      }
      broadcast({pData: pData});
    }
  }
  while (this.totalIterations < this.maxIterations);

  this.graphManager.updateBounds();
};

CoSELayout.prototype.calculateNodesToApplyGravitationTo = function () {
  var nodeList = [];
  var graph;

  var graphs = this.graphManager.getGraphs();
  var size = graphs.length;
  var i;
  for (i = 0; i < size; i++)
  {
    graph = graphs[i];

    graph.updateConnected();

    if (!graph.isConnected)
    {
      nodeList = nodeList.concat(graph.getNodes());
    }
  }

  this.graphManager.setAllNodesToApplyGravitation(nodeList);
};

CoSELayout.prototype.createBendpoints = function () {
  var edges = [];
  edges = edges.concat(this.graphManager.getAllEdges());
  var visited = new HashSet();
  var i;
  for (i = 0; i < edges.length; i++)
  {
    var edge = edges[i];

    if (!visited.contains(edge))
    {
      var source = edge.getSource();
      var target = edge.getTarget();

      if (source == target)
      {
        edge.getBendpoints().push(new PointD());
        edge.getBendpoints().push(new PointD());
        this.createDummyNodesForBendpoints(edge);
        visited.add(edge);
      }
      else
      {
        var edgeList = [];

        edgeList = edgeList.concat(source.getEdgeListToNode(target));
        edgeList = edgeList.concat(target.getEdgeListToNode(source));

        if (!visited.contains(edgeList[0]))
        {
          if (edgeList.length > 1)
          {
            var k;
            for (k = 0; k < edgeList.length; k++)
            {
              var multiEdge = edgeList[k];
              multiEdge.getBendpoints().push(new PointD());
              this.createDummyNodesForBendpoints(multiEdge);
            }
          }
          visited.addAll(list);
        }
      }
    }

    if (visited.size() == edges.length)
    {
      break;
    }
  }
};

CoSELayout.prototype.positionNodesRadially = function (forest) {
  // We tile the trees to a grid row by row; first tree starts at (0,0)
  var currentStartingPoint = new Point(0, 0);
  var numberOfColumns = Math.ceil(Math.sqrt(forest.length));
  var height = 0;
  var currentY = 0;
  var currentX = 0;
  var point = new PointD(0, 0);

  for (var i = 0; i < forest.length; i++)
  {
    if (i % numberOfColumns == 0)
    {
      // Start of a new row, make the x coordinate 0, increment the
      // y coordinate with the max height of the previous row
      currentX = 0;
      currentY = height;

      if (i != 0)
      {
        currentY += CoSEConstants.DEFAULT_COMPONENT_SEPERATION;
      }

      height = 0;
    }

    var tree = forest[i];

    // Find the center of the tree
    var centerNode = Layout.findCenterOfTree(tree);

    // Set the staring point of the next tree
    currentStartingPoint.x = currentX;
    currentStartingPoint.y = currentY;

    // Do a radial layout starting with the center
    point =
            CoSELayout.radialLayout(tree, centerNode, currentStartingPoint);

    if (point.y > height)
    {
      height = Math.floor(point.y);
    }

    currentX = Math.floor(point.x + CoSEConstants.DEFAULT_COMPONENT_SEPERATION);
  }

  this.transform(
          new PointD(LayoutConstants.WORLD_CENTER_X - point.x / 2,
                  LayoutConstants.WORLD_CENTER_Y - point.y / 2));
};

CoSELayout.radialLayout = function (tree, centerNode, startingPoint) {
  var radialSep = Math.max(this.maxDiagonalInTree(tree),
          CoSEConstants.DEFAULT_RADIAL_SEPARATION);
  CoSELayout.branchRadialLayout(centerNode, null, 0, 359, 0, radialSep);
  var bounds = LGraph.calculateBounds(tree);

  var transform = new Transform();
  transform.setDeviceOrgX(bounds.getMinX());
  transform.setDeviceOrgY(bounds.getMinY());
  transform.setWorldOrgX(startingPoint.x);
  transform.setWorldOrgY(startingPoint.y);

  for (var i = 0; i < tree.length; i++)
  {
    var node = tree[i];
    node.transform(transform);
  }

  var bottomRight =
          new PointD(bounds.getMaxX(), bounds.getMaxY());

  return transform.inverseTransformPoint(bottomRight);
};

CoSELayout.branchRadialLayout = function (node, parentOfNode, startAngle, endAngle, distance, radialSeparation) {
  // First, position this node by finding its angle.
  var halfInterval = ((endAngle - startAngle) + 1) / 2;

  if (halfInterval < 0)
  {
    halfInterval += 180;
  }

  var nodeAngle = (halfInterval + startAngle) % 360;
  var teta = (nodeAngle * IGeometry.TWO_PI) / 360;

  // Make polar to java cordinate conversion.
  var cos_teta = Math.cos(teta);
  var x_ = distance * Math.cos(teta);
  var y_ = distance * Math.sin(teta);

  node.setCenter(x_, y_);

  // Traverse all neighbors of this node and recursively call this
  // function.
  var neighborEdges = [];
  neighborEdges = neighborEdges.concat(node.getEdges());
  var childCount = neighborEdges.length;

  if (parentOfNode != null)
  {
    childCount--;
  }

  var branchCount = 0;

  var incEdgesCount = neighborEdges.length;
  var startIndex;

  var edges = node.getEdgesBetween(parentOfNode);

  // If there are multiple edges, prune them until there remains only one
  // edge.
  while (edges.length > 1)
  {
    //neighborEdges.remove(edges.remove(0));
    var temp = edges[0];
    edges.splice(0, 1);
    var index = neighborEdges.indexOf(temp);
    if (index >= 0) {
      neighborEdges.splice(index, 1);
    }
    incEdgesCount--;
    childCount--;
  }

  if (parentOfNode != null)
  {
    //assert edges.length == 1;
    startIndex = (neighborEdges.indexOf(edges[0]) + 1) % incEdgesCount;
  }
  else
  {
    startIndex = 0;
  }

  var stepAngle = Math.abs(endAngle - startAngle) / childCount;

  for (var i = startIndex;
          branchCount != childCount;
          i = (++i) % incEdgesCount)
  {
    var currentNeighbor =
            neighborEdges[i].getOtherEnd(node);

    // Don't back traverse to root node in current tree.
    if (currentNeighbor == parentOfNode)
    {
      continue;
    }

    var childStartAngle =
            (startAngle + branchCount * stepAngle) % 360;
    var childEndAngle = (childStartAngle + stepAngle) % 360;

    CoSELayout.branchRadialLayout(currentNeighbor,
            node,
            childStartAngle, childEndAngle,
            distance + radialSeparation, radialSeparation);

    branchCount++;
  }
};

CoSELayout.maxDiagonalInTree = function (tree) {
  var maxDiagonal = Integer.MIN_VALUE;

  for (var i = 0; i < tree.length; i++)
  {
    var node = tree[i];
    var diagonal = node.getDiagonal();

    if (diagonal > maxDiagonal)
    {
      maxDiagonal = diagonal;
    }
  }

  return maxDiagonal;
};

CoSELayout.prototype.calcRepulsionRange = function () {
  // formula is 2 x (level + 1) x idealEdgeLength
  return (2 * (this.level + 1) * this.idealEdgeLength);
};

module.exports = CoSELayout;

},{"./CoSEEdge":2,"./CoSEGraph":3,"./CoSEGraphManager":4,"./CoSENode":6,"./FDLayout":9}],6:[function(_dereq_,module,exports){
var FDLayoutNode = _dereq_('./FDLayoutNode');

function CoSENode(gm, loc, size, vNode) {
  FDLayoutNode.call(this, gm, loc, size, vNode);
}


CoSENode.prototype = Object.create(FDLayoutNode.prototype);
for (var prop in FDLayoutNode) {
  CoSENode[prop] = FDLayoutNode[prop];
}

CoSENode.prototype.move = function ()
{
  var layout = this.graphManager.getLayout();
  this.displacementX = layout.coolingFactor *
          (this.springForceX + this.repulsionForceX + this.gravitationForceX);
  this.displacementY = layout.coolingFactor *
          (this.springForceY + this.repulsionForceY + this.gravitationForceY);


  if (Math.abs(this.displacementX) > layout.coolingFactor * layout.maxNodeDisplacement)
  {
    this.displacementX = layout.coolingFactor * layout.maxNodeDisplacement *
            IMath.sign(this.displacementX);
  }

  if (Math.abs(this.displacementY) > layout.coolingFactor * layout.maxNodeDisplacement)
  {
    this.displacementY = layout.coolingFactor * layout.maxNodeDisplacement *
            IMath.sign(this.displacementY);
  }

  // a simple node, just move it
  if (this.child == null)
  {
    this.moveBy(this.displacementX, this.displacementY);
  }
  // an empty compound node, again just move it
  else if (this.child.getNodes().length == 0)
  {
    this.moveBy(this.displacementX, this.displacementY);
  }
  // non-empty compound node, propogate movement to children as well
  else
  {
    this.propogateDisplacementToChildren(this.displacementX,
            this.displacementY);
  }

  layout.totalDisplacement +=
          Math.abs(this.displacementX) + Math.abs(this.displacementY);

  this.springForceX = 0;
  this.springForceY = 0;
  this.repulsionForceX = 0;
  this.repulsionForceY = 0;
  this.gravitationForceX = 0;
  this.gravitationForceY = 0;
  this.displacementX = 0;
  this.displacementY = 0;
};

CoSENode.prototype.propogateDisplacementToChildren = function (dX, dY)
{
  var nodes = this.getChild().getNodes();
  var node;
  for (var i = 0; i < nodes.length; i++)
  {
    node = nodes[i];
    if (node.getChild() == null)
    {
      node.moveBy(dX, dY);
      node.displacementX += dX;
      node.displacementY += dY;
    }
    else
    {
      node.propogateDisplacementToChildren(dX, dY);
    }
  }
};

CoSENode.prototype.setPred1 = function (pred1)
{
  this.pred1 = pred1;
};

CoSENode.prototype.getPred1 = function ()
{
  return pred1;
};

CoSENode.prototype.getPred2 = function ()
{
  return pred2;
};

CoSENode.prototype.setNext = function (next)
{
  this.next = next;
};

CoSENode.prototype.getNext = function ()
{
  return next;
};

CoSENode.prototype.setProcessed = function (processed)
{
  this.processed = processed;
};

CoSENode.prototype.isProcessed = function ()
{
  return processed;
};

module.exports = CoSENode;

},{"./FDLayoutNode":12}],7:[function(_dereq_,module,exports){
var VisibilityEdge = _dereq_('./VisibilityEdge');
var VisibilityGraph = _dereq_('./VisibilityGraph');
var SbgnPDConstants = _dereq_('./SbgnPDConstants');

function Compaction(vertices) 
{
    this.orderedNodeList = [] /*ArrayList<SbgnPDNode>()*/;
    this.vertices = vertices;
    
    this.visGraph = null;
    this.direction = null;
}

Compaction.prototype.CompactionDirectionEnum = 
{
    VERTICAL : 0, 
    HORIZONTAL : 1
};

/**
* Two times do the following: (first for vertical, second horizontal) First
* create a visibility graph for the given elements. The visibility graph is
* always a DAG, so perform a topological sort on the elements (the node
* that has in-degree 0 comes first), perform compaction.
*/
Compaction.prototype.perform = function ()
{
    this.algorithmBody(this.CompactionDirectionEnum.VERTICAL);
    this.removeVisibilityEdges();

    this.algorithmBody(this.CompactionDirectionEnum.HORIZONTAL);
    this.removeVisibilityEdges();

};

Compaction.prototype.removeVisibilityEdges = function ()
{
    var numOfVertices = this.vertices.length;
    for(var i=0; i<numOfVertices; i++)
    {
        var sbgnNode = this.vertices[i];

        for(var j = 0; j < sbgnNode.getEdges().length; j++)
        {
            var objEdge = sbgnNode.getEdges()[j];

            if( objEdge instanceof VisibilityEdge)
            {
                sbgnNode.getEdges().splice(i, 1);
                i--;
            }
        }
    }	
};

Compaction.prototype.algorithmBody = function (direction)
{
    this.direction = direction;
    this.visGraph = new VisibilityGraph(null, null, null);

    // construct a visibility graph given the direction and vertices
    this.visGraph.construct(this.direction, vertices);

    if (this.visGraph.getEdges().length > 0)
    {
        this.topologicallySort();
        this.compactElements();
    }

    // positions of the vertices has changed. Update them.
    this.vertices = this.visGraph.getNodes();
};

/**
* Perform a DFS on the given graph nodes and then output the nodes in
* reverse order.
*/
Compaction.prototype.topologicallySort = function ()
{
    // ensure that the vertices have not been marked as visited.
    for (var i = 0; i < this.visGraph.getNodes().length; i++)
    {
        var s = this.visGraph.getNodes()[i];
        s.visited = false;
    }

    // ensure that the list is empty
    this.orderedNodeList = [];
    this.DFS();
    this.orderedNodeList = this.reverseList(this.orderedNodeList);
};

Compaction.prototype.DFS = function ()
{
    for (var i = 0; i < this.visGraph.getNodes().length; i++)
    {
        var s = this.visGraph.getNodes()[i];
        if (!s.visited)
        {
            this.DFS_Visit(s);
        }
    }
};

Compaction.prototype.DFS_Visit = function (s)
{
    var neighbors = s.getChildrenNeighbors(null);

    if (neighbors.length === 0)
    {
        s.visited = true;
        this.orderedNodeList.push(s);
        return;
    }

    var numOfNeighbours = neighbors.length;
    for (var i=0; i<numOfNeighbours; i++)
    {
        if (!neighbors[i].visited)
        {
            this.DFS_Visit(neighbors[i]);
        }
    }

    s.visited = true;
    this.orderedNodeList.push(s);
};

/**
* Reverse the element order of a given list
*/
Compaction.prototype.reverseList = function (originalList)
{
    var reverseOutput = [];
    for (var i = originalList.length - 1; i >= 0; i--)
    {
        reverseOutput.push(originalList[i]);
    }

    return reverseOutput;
};

/**
* This method visits the list that is the result of topological sort. For
* each node in that list, it looks for its incoming edges and finds the
* shortest one. Translates the node wrt the shortest edge.
*/
Compaction.prototype.compactElements = function ()
{
    var distance = 0.0;
    
    var orderedNodeListLength = this.orderedNodeList.length;
    for (var i=0; i<orderedNodeListLength; i++)
    {
        var sbgnPDNode = this.orderedNodeList[i];
        
        // find shortest incoming edge
        var edge = this.visGraph.findShortestEdge(sbgnPDNode);

        if (edge != null)
        {
            distance = edge.getLength();

            if (this.direction === this.CompactionDirectionEnum.VERTICAL)
            {
                // bring the node closer to the source node and respect the
                // buffer.
                if (distance > SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER)
                {
                    sbgnPDNode.setLocation(
                            sbgnPDNode.getLeft(),
                            (sbgnPDNode.getTop() - 
                                    (distance - SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER)));
                }
                else
                {
                    sbgnPDNode.setLocation(
                            sbgnPDNode.getLeft(), 
                            edge.getOtherEnd(sbgnPDNode).getBottom()
                                    + SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER);
                }
            }
            else if (this.direction === this.CompactionDirectionEnum.HORIZONTAL)
            {
                if (distance > SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER)
                {
                    sbgnPDNode.setLocation(
                            (sbgnPDNode.getLeft() - (distance - SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER)),
                            sbgnPDNode.getTop());
                }
                else
                {
                    sbgnPDNode.setLocation(
                            edge.getOtherEnd(sbgnPDNode).getRight()
                                            + SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER,
                            sbgnPDNode.getTop());
                }
            }
        }
    }
};

module.exports = Compaction;



},{"./SbgnPDConstants":35,"./VisibilityEdge":42,"./VisibilityGraph":43}],8:[function(_dereq_,module,exports){
function DimensionD(width, height) {
  this.width = 0;
  this.height = 0;
  if (width !== null && height !== null) {
    this.height = height;
    this.width = width;
  }
}

DimensionD.prototype.getWidth = function ()
{
  return this.width;
};

DimensionD.prototype.setWidth = function (width)
{
  this.width = width;
};

DimensionD.prototype.getHeight = function ()
{
  return this.height;
};

DimensionD.prototype.setHeight = function (height)
{
  this.height = height;
};

module.exports = DimensionD;

},{}],9:[function(_dereq_,module,exports){
var Layout = _dereq_('./Layout');
var FDLayoutConstants = _dereq_('./FDLayoutConstants');

function FDLayout() {
  Layout.call(this);

  this.useSmartIdealEdgeLengthCalculation = FDLayoutConstants.DEFAULT_USE_SMART_IDEAL_EDGE_LENGTH_CALCULATION;
  this.idealEdgeLength = FDLayoutConstants.DEFAULT_EDGE_LENGTH;
  this.springConstant = FDLayoutConstants.DEFAULT_SPRING_STRENGTH;
  this.repulsionConstant = FDLayoutConstants.DEFAULT_REPULSION_STRENGTH;
  this.gravityConstant = FDLayoutConstants.DEFAULT_GRAVITY_STRENGTH;
  this.compoundGravityConstant = FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_STRENGTH;
  this.gravityRangeFactor = FDLayoutConstants.DEFAULT_GRAVITY_RANGE_FACTOR;
  this.compoundGravityRangeFactor = FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_RANGE_FACTOR;
  this.displacementThresholdPerNode = (3.0 * FDLayoutConstants.DEFAULT_EDGE_LENGTH) / 100;
  this.coolingFactor = 1.0;
  this.initialCoolingFactor = 1.0;
  this.totalDisplacement = 0.0;
  this.oldTotalDisplacement = 0.0;
  this.maxIterations = FDLayoutConstants.MAX_ITERATIONS;
}

FDLayout.prototype = Object.create(Layout.prototype);

for (var prop in Layout) {
  FDLayout[prop] = Layout[prop];
}

FDLayout.prototype.initParameters = function () {
  Layout.prototype.initParameters.call(this, arguments);

  if (this.layoutQuality == LayoutConstants.DRAFT_QUALITY)
  {
    this.displacementThresholdPerNode += 0.30;
    this.maxIterations *= 0.8;
  }
  else if (this.layoutQuality == LayoutConstants.PROOF_QUALITY)
  {
    this.displacementThresholdPerNode -= 0.30;
    this.maxIterations *= 1.2;
  }

  this.totalIterations = 0;
  this.notAnimatedIterations = 0;

//    this.useFRGridVariant = layoutOptionsPack.smartRepulsionRangeCalc;
};

FDLayout.prototype.calcIdealEdgeLengths = function () {
  var edge;
  var lcaDepth;
  var source;
  var target;
  var sizeOfSourceInLca;
  var sizeOfTargetInLca;

  var allEdges = this.getGraphManager().getAllEdges();
  for (var i = 0; i < allEdges.length; i++)
  {
    edge = allEdges[i];

    edge.idealLength = this.idealEdgeLength;

    if (edge.isInterGraph)
    {
      source = edge.getSource();
      target = edge.getTarget();

      sizeOfSourceInLca = edge.getSourceInLca().getEstimatedSize();
      sizeOfTargetInLca = edge.getTargetInLca().getEstimatedSize();

      if (this.useSmartIdealEdgeLengthCalculation)
      {
        edge.idealLength += sizeOfSourceInLca + sizeOfTargetInLca -
                2 * LayoutConstants.SIMPLE_NODE_SIZE;
      }

      lcaDepth = edge.getLca().getInclusionTreeDepth();

      edge.idealLength += FDLayoutConstants.DEFAULT_EDGE_LENGTH *
              FDLayoutConstants.PER_LEVEL_IDEAL_EDGE_LENGTH_FACTOR *
              (source.getInclusionTreeDepth() +
                      target.getInclusionTreeDepth() - 2 * lcaDepth);
    }
  }
};

FDLayout.prototype.initSpringEmbedder = function () {

  if (this.incremental)
  {
    this.coolingFactor = 0.8;
    this.initialCoolingFactor = 0.8;
    this.maxNodeDisplacement =
            FDLayoutConstants.MAX_NODE_DISPLACEMENT_INCREMENTAL;
  }
  else
  {
    this.coolingFactor = 1.0;
    this.initialCoolingFactor = 1.0;
    this.maxNodeDisplacement =
            FDLayoutConstants.MAX_NODE_DISPLACEMENT;
  }

  this.maxIterations =
          Math.max(this.getAllNodes().length * 5, this.maxIterations);

  this.totalDisplacementThreshold =
          this.displacementThresholdPerNode * this.getAllNodes().length;

  this.repulsionRange = this.calcRepulsionRange();
};

FDLayout.prototype.calcSpringForces = function () {
  var lEdges = this.getAllEdges();
  var edge;

  for (var i = 0; i < lEdges.length; i++)
  {
    edge = lEdges[i];

    this.calcSpringForce(edge, edge.idealLength);
  }
};

FDLayout.prototype.calcRepulsionForces = function () {
  var i, j;
  var nodeA, nodeB;
  var lNodes = this.getAllNodes();

  for (i = 0; i < lNodes.length; i++)
  {
    nodeA = lNodes[i];

    for (j = i + 1; j < lNodes.length; j++)
    {
      nodeB = lNodes[j];

      // If both nodes are not members of the same graph, skip.
      if (nodeA.getOwner() != nodeB.getOwner())
      {
        continue;
      }

      this.calcRepulsionForce(nodeA, nodeB);
    }
  }
};

FDLayout.prototype.calcGravitationalForces = function () {
  var node;
  var lNodes = this.getAllNodesToApplyGravitation();

  for (var i = 0; i < lNodes.length; i++)
  {
    node = lNodes[i];
    this.calcGravitationalForce(node);
  }
};

FDLayout.prototype.moveNodes = function () {
  var lNodes = this.getAllNodes();
  var node;

  for (var i = 0; i < lNodes.length; i++)
  {
    node = lNodes[i];
    node.move();
  }
}

FDLayout.prototype.calcSpringForce = function (edge, idealLength) {
  var sourceNode = edge.getSource();
  var targetNode = edge.getTarget();

  var length;
  var springForce;
  var springForceX;
  var springForceY;

  // Update edge length
  if (this.uniformLeafNodeSizes &&
          sourceNode.getChild() == null && targetNode.getChild() == null)
  {
    edge.updateLengthSimple();
  }
  else
  {
    edge.updateLength();

    if (edge.isOverlapingSourceAndTarget)
    {
      return;
    }
  }

  length = edge.getLength();

  // Calculate spring forces
  springForce = this.springConstant * (length - idealLength);

  // Project force onto x and y axes
  springForceX = springForce * (edge.lengthX / length);
  springForceY = springForce * (edge.lengthY / length);

  // Apply forces on the end nodes
  sourceNode.springForceX += springForceX;
  sourceNode.springForceY += springForceY;
  targetNode.springForceX -= springForceX;
  targetNode.springForceY -= springForceY;
};

FDLayout.prototype.calcRepulsionForce = function (nodeA, nodeB) {
  var rectA = nodeA.getRect();
  var rectB = nodeB.getRect();
  var overlapAmount = new Array(2);
  var clipPoints = new Array(4);
  var distanceX;
  var distanceY;
  var distanceSquared;
  var distance;
  var repulsionForce;
  var repulsionForceX;
  var repulsionForceY;

  if (rectA.intersects(rectB))// two nodes overlap
  {
    // calculate separation amount in x and y directions
    IGeometry.calcSeparationAmount(rectA,
            rectB,
            overlapAmount,
            FDLayoutConstants.DEFAULT_EDGE_LENGTH / 2.0);

    repulsionForceX = overlapAmount[0];
    repulsionForceY = overlapAmount[1];
  }
  else// no overlap
  {
    // calculate distance

    if (this.uniformLeafNodeSizes &&
            nodeA.getChild() == null && nodeB.getChild() == null)// simply base repulsion on distance of node centers
    {
      distanceX = rectB.getCenterX() - rectA.getCenterX();
      distanceY = rectB.getCenterY() - rectA.getCenterY();
    }
    else// use clipping points
    {
      IGeometry.getIntersection(rectA, rectB, clipPoints);

      distanceX = clipPoints[2] - clipPoints[0];
      distanceY = clipPoints[3] - clipPoints[1];
    }

    // No repulsion range. FR grid variant should take care of this.
    if (Math.abs(distanceX) < FDLayoutConstants.MIN_REPULSION_DIST)
    {
      distanceX = IMath.sign(distanceX) *
              FDLayoutConstants.MIN_REPULSION_DIST;
    }

    if (Math.abs(distanceY) < FDLayoutConstants.MIN_REPULSION_DIST)
    {
      distanceY = IMath.sign(distanceY) *
              FDLayoutConstants.MIN_REPULSION_DIST;
    }

    distanceSquared = distanceX * distanceX + distanceY * distanceY;
    distance = Math.sqrt(distanceSquared);

    repulsionForce = this.repulsionConstant / distanceSquared;

    // Project force onto x and y axes
    repulsionForceX = repulsionForce * distanceX / distance;
    repulsionForceY = repulsionForce * distanceY / distance;
  }

  // Apply forces on the two nodes
  nodeA.repulsionForceX -= repulsionForceX;
  nodeA.repulsionForceY -= repulsionForceY;
  nodeB.repulsionForceX += repulsionForceX;
  nodeB.repulsionForceY += repulsionForceY;
};

FDLayout.prototype.calcGravitationalForce = function (node) {
  var ownerGraph;
  var ownerCenterX;
  var ownerCenterY;
  var distanceX;
  var distanceY;
  var absDistanceX;
  var absDistanceY;
  var estimatedSize;
  ownerGraph = node.getOwner();

  ownerCenterX = (ownerGraph.getRight() + ownerGraph.getLeft()) / 2;
  ownerCenterY = (ownerGraph.getTop() + ownerGraph.getBottom()) / 2;
  distanceX = node.getCenterX() - ownerCenterX;
  distanceY = node.getCenterY() - ownerCenterY;
  absDistanceX = Math.abs(distanceX);
  absDistanceY = Math.abs(distanceY);

  if (node.getOwner() == this.graphManager.getRoot())// in the root graph
  {
    Math.floor(80);
    estimatedSize = Math.floor(ownerGraph.getEstimatedSize() *
            this.gravityRangeFactor);

    if (absDistanceX > estimatedSize || absDistanceY > estimatedSize)
    {
      node.gravitationForceX = -this.gravityConstant * distanceX;
      node.gravitationForceY = -this.gravityConstant * distanceY;
    }
  }
  else// inside a compound
  {
    estimatedSize = Math.floor((ownerGraph.getEstimatedSize() *
            this.compoundGravityRangeFactor));

    if (absDistanceX > estimatedSize || absDistanceY > estimatedSize)
    {
      node.gravitationForceX = -this.gravityConstant * distanceX *
              this.compoundGravityConstant;
      node.gravitationForceY = -this.gravityConstant * distanceY *
              this.compoundGravityConstant;
    }
  }
};

FDLayout.prototype.isConverged = function () {
  var converged;
  var oscilating = false;

  if (this.totalIterations > this.maxIterations / 3)
  {
    oscilating =
            Math.abs(this.totalDisplacement - this.oldTotalDisplacement) < 2;
  }

  converged = this.totalDisplacement < this.totalDisplacementThreshold;

  this.oldTotalDisplacement = this.totalDisplacement;

  return converged || oscilating;
};

FDLayout.prototype.animate = function () {
  if (this.animationDuringLayout && !this.isSubLayout)
  {
    if (this.notAnimatedIterations == this.animationPeriod)
    {
      this.update();
      this.notAnimatedIterations = 0;
    }
    else
    {
      this.notAnimatedIterations++;
    }
  }
};

FDLayout.prototype.calcRepulsionRange = function () {
  return 0.0;
};

module.exports = FDLayout;

},{"./FDLayoutConstants":10,"./Layout":23}],10:[function(_dereq_,module,exports){
var LayoutConstants = _dereq_('./LayoutConstants');

function FDLayoutConstants() {
}

//FDLayoutConstants inherits static props in LayoutConstants
for (var prop in LayoutConstants) {
  FDLayoutConstants[prop] = LayoutConstants[prop];
}

FDLayoutConstants.MAX_ITERATIONS = 2500;

FDLayoutConstants.DEFAULT_EDGE_LENGTH = 50;
FDLayoutConstants.DEFAULT_SPRING_STRENGTH = 0.45;
FDLayoutConstants.DEFAULT_REPULSION_STRENGTH = 4500.0;
FDLayoutConstants.DEFAULT_GRAVITY_STRENGTH = 0.4;
FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_STRENGTH = 1.0;
FDLayoutConstants.DEFAULT_GRAVITY_RANGE_FACTOR = 3.8;
FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_RANGE_FACTOR = 1.5;
FDLayoutConstants.DEFAULT_USE_SMART_IDEAL_EDGE_LENGTH_CALCULATION = true;
FDLayoutConstants.DEFAULT_USE_SMART_REPULSION_RANGE_CALCULATION = true;
FDLayoutConstants.MAX_NODE_DISPLACEMENT_INCREMENTAL = 100.0;
FDLayoutConstants.MAX_NODE_DISPLACEMENT = FDLayoutConstants.MAX_NODE_DISPLACEMENT_INCREMENTAL * 3;
FDLayoutConstants.MIN_REPULSION_DIST = FDLayoutConstants.DEFAULT_EDGE_LENGTH / 10.0;
FDLayoutConstants.CONVERGENCE_CHECK_PERIOD = 100;
FDLayoutConstants.PER_LEVEL_IDEAL_EDGE_LENGTH_FACTOR = 0.1;
FDLayoutConstants.MIN_EDGE_LENGTH = 1;
FDLayoutConstants.GRID_CALCULATION_CHECK_PERIOD = 10;

module.exports = FDLayoutConstants;

},{"./LayoutConstants":24}],11:[function(_dereq_,module,exports){
var LEdge = _dereq_('./LEdge');
var FDLayoutConstants = _dereq_('./FDLayoutConstants');

function FDLayoutEdge(source, target, vEdge) {
  LEdge.call(this, source, target, vEdge);
  this.idealLength = FDLayoutConstants.DEFAULT_EDGE_LENGTH;
}

FDLayoutEdge.prototype = Object.create(LEdge.prototype);

for (var prop in LEdge) {
  FDLayoutEdge[prop] = LEdge[prop];
}

module.exports = FDLayoutEdge;

},{"./FDLayoutConstants":10,"./LEdge":18}],12:[function(_dereq_,module,exports){
var LNode = _dereq_('./LNode');

function FDLayoutNode(gm, loc, size, vNode) {
  // alternative constructor is handled inside LNode
  LNode.call(this, gm, loc, size, vNode);
  //Spring, repulsion and gravitational forces acting on this node
  this.springForceX = 0;
  this.springForceY = 0;
  this.repulsionForceX = 0;
  this.repulsionForceY = 0;
  this.gravitationForceX = 0;
  this.gravitationForceY = 0;
  //Amount by which this node is to be moved in this iteration
  this.displacementX = 0;
  this.displacementY = 0;

  //Start and finish grid coordinates that this node is fallen into
  this.startX = 0;
  this.finishX = 0;
  this.startY = 0;
  this.finishY = 0;

  //Geometric neighbors of this node
  this.surrounding = [];
}

FDLayoutNode.prototype = Object.create(LNode.prototype);

for (var prop in LNode) {
  FDLayoutNode[prop] = LNode[prop];
}

FDLayoutNode.prototype.setGridCoordinates = function (_startX, _finishX, _startY, _finishY)
{
  this.startX = _startX;
  this.finishX = _finishX;
  this.startY = _startY;
  this.finishY = _finishY;

};

module.exports = FDLayoutNode;

},{"./LNode":22}],13:[function(_dereq_,module,exports){
var UniqueIDGeneretor = _dereq_('./UniqueIDGeneretor');

function HashMap() {
  this.map = {};
  this.keys = [];
}

HashMap.prototype.put = function (key, value) {
  var theId = UniqueIDGeneretor.createID(key);
  if (!this.contains(theId)) {
    this.map[theId] = value;
    this.keys.push(key);
  }
};

HashMap.prototype.contains = function (key) {
  var theId = UniqueIDGeneretor.createID(key);
  return this.map[key] != null;
};

HashMap.prototype.get = function (key) {
  var theId = UniqueIDGeneretor.createID(key);
  return this.map[theId];
};

HashMap.prototype.keySet = function () {
  return this.keys;
};

module.exports = HashMap;

},{"./UniqueIDGeneretor":41}],14:[function(_dereq_,module,exports){
var UniqueIDGeneretor = _dereq_('./UniqueIDGeneretor');

function HashSet() {
  this.set = {};
}
;

HashSet.prototype.add = function (obj) {
  var theId = UniqueIDGeneretor.createID(obj);
  if (!this.contains(theId))
    this.set[theId] = obj;
};

HashSet.prototype.remove = function (obj) {
  delete this.set[UniqueIDGeneretor.createID(obj)];
};

HashSet.prototype.clear = function () {
  this.set = {};
};

HashSet.prototype.contains = function (obj) {
  return this.set[UniqueIDGeneretor.createID(obj)] == obj;
};

HashSet.prototype.isEmpty = function () {
  return this.size() === 0;
};

HashSet.prototype.size = function () {
  return Object.keys(this.set).length;
};

//concats this.set to the given list
HashSet.prototype.addAllTo = function (list) {
  var keys = Object.keys(this.set);
  var length = keys.length;
  for (var i = 0; i < length; i++) {
    list.push(this.set[keys[i]]);
  }
};

HashSet.prototype.size = function () {
  return Object.keys(this.set).length;
};

HashSet.prototype.addAll = function (list) {
  var s = list.length;
  for (var i = 0; i < s; i++) {
    var v = list[i];
    this.add(v);
  }
};

module.exports = HashSet;

},{"./UniqueIDGeneretor":41}],15:[function(_dereq_,module,exports){
function IGeometry() {
}

IGeometry.calcSeparationAmount = function (rectA, rectB, overlapAmount, separationBuffer)
{
  if (!rectA.intersects(rectB)) {
    throw "assert failed";
  }
  var directions = new Array(2);
  IGeometry.decideDirectionsForOverlappingNodes(rectA, rectB, directions);
  overlapAmount[0] = Math.min(rectA.getRight(), rectB.getRight()) -
          Math.max(rectA.x, rectB.x);
  overlapAmount[1] = Math.min(rectA.getBottom(), rectB.getBottom()) -
          Math.max(rectA.y, rectB.y);
  // update the overlapping amounts for the following cases:
  if ((rectA.getX() <= rectB.getX()) && (rectA.getRight() >= rectB.getRight()))
  {
    overlapAmount[0] += Math.min((rectB.getX() - rectA.getX()),
            (rectA.getRight() - rectB.getRight()));
  }
  else if ((rectB.getX() <= rectA.getX()) && (rectB.getRight() >= rectA.getRight()))
  {
    overlapAmount[0] += Math.min((rectA.getX() - rectB.getX()),
            (rectB.getRight() - rectA.getRight()));
  }
  if ((rectA.getY() <= rectB.getY()) && (rectA.getBottom() >= rectB.getBottom()))
  {
    overlapAmount[1] += Math.min((rectB.getY() - rectA.getY()),
            (rectA.getBottom() - rectB.getBottom()));
  }
  else if ((rectB.getY() <= rectA.getY()) && (rectB.getBottom() >= rectA.getBottom()))
  {
    overlapAmount[1] += Math.min((rectA.getY() - rectB.getY()),
            (rectB.getBottom() - rectA.getBottom()));
  }

  // find slope of the line passes two centers
  var slope = Math.abs((rectB.getCenterY() - rectA.getCenterY()) /
          (rectB.getCenterX() - rectA.getCenterX()));
  // if centers are overlapped
  if ((rectB.getCenterY() == rectA.getCenterY()) &&
          (rectB.getCenterX() == rectA.getCenterX()))
  {
    // assume the slope is 1 (45 degree)
    slope = 1.0;
  }

  var moveByY = slope * overlapAmount[0];
  var moveByX = overlapAmount[1] / slope;
  if (overlapAmount[0] < moveByX)
  {
    moveByX = overlapAmount[0];
  }
  else
  {
    moveByY = overlapAmount[1];
  }
  // return half the amount so that if each rectangle is moved by these
  // amounts in opposite directions, overlap will be resolved
  overlapAmount[0] = -1 * directions[0] * ((moveByX / 2) + separationBuffer);
  overlapAmount[1] = -1 * directions[1] * ((moveByY / 2) + separationBuffer);
}

IGeometry.decideDirectionsForOverlappingNodes = function (rectA, rectB, directions)
{
  if (rectA.getCenterX() < rectB.getCenterX())
  {
    directions[0] = -1;
  }
  else
  {
    directions[0] = 1;
  }

  if (rectA.getCenterY() < rectB.getCenterY())
  {
    directions[1] = -1;
  }
  else
  {
    directions[1] = 1;
  }
}

IGeometry.getIntersection2 = function (rectA, rectB, result)
{
  //result[0-1] will contain clipPoint of rectA, result[2-3] will contain clipPoint of rectB
  var p1x = rectA.getCenterX();
  var p1y = rectA.getCenterY();
  var p2x = rectB.getCenterX();
  var p2y = rectB.getCenterY();

  //if two rectangles intersect, then clipping points are centers
  if (rectA.intersects(rectB))
  {
    result[0] = p1x;
    result[1] = p1y;
    result[2] = p2x;
    result[3] = p2y;
    return true;
  }
  //variables for rectA
  var topLeftAx = rectA.getX();
  var topLeftAy = rectA.getY();
  var topRightAx = rectA.getRight();
  var bottomLeftAx = rectA.getX();
  var bottomLeftAy = rectA.getBottom();
  var bottomRightAx = rectA.getRight();
  var halfWidthA = rectA.getWidthHalf();
  var halfHeightA = rectA.getHeightHalf();
  //variables for rectB
  var topLeftBx = rectB.getX();
  var topLeftBy = rectB.getY();
  var topRightBx = rectB.getRight();
  var bottomLeftBx = rectB.getX();
  var bottomLeftBy = rectB.getBottom();
  var bottomRightBx = rectB.getRight();
  var halfWidthB = rectB.getWidthHalf();
  var halfHeightB = rectB.getHeightHalf();
  //flag whether clipping points are found
  var clipPointAFound = false;
  var clipPointBFound = false;

  // line is vertical
  if (p1x == p2x)
  {
    if (p1y > p2y)
    {
      result[0] = p1x;
      result[1] = topLeftAy;
      result[2] = p2x;
      result[3] = bottomLeftBy;
      return false;
    }
    else if (p1y < p2y)
    {
      result[0] = p1x;
      result[1] = bottomLeftAy;
      result[2] = p2x;
      result[3] = topLeftBy;
      return false;
    }
    else
    {
      //not line, return null;
    }
  }
  // line is horizontal
  else if (p1y == p2y)
  {
    if (p1x > p2x)
    {
      result[0] = topLeftAx;
      result[1] = p1y;
      result[2] = topRightBx;
      result[3] = p2y;
      return false;
    }
    else if (p1x < p2x)
    {
      result[0] = topRightAx;
      result[1] = p1y;
      result[2] = topLeftBx;
      result[3] = p2y;
      return false;
    }
    else
    {
      //not valid line, return null;
    }
  }
  else
  {
    //slopes of rectA's and rectB's diagonals
    var slopeA = rectA.height / rectA.width;
    var slopeB = rectB.height / rectB.width;

    //slope of line between center of rectA and center of rectB
    var slopePrime = (p2y - p1y) / (p2x - p1x);
    var cardinalDirectionA;
    var cardinalDirectionB;
    var tempPointAx;
    var tempPointAy;
    var tempPointBx;
    var tempPointBy;

    //determine whether clipping point is the corner of nodeA
    if ((-slopeA) == slopePrime)
    {
      if (p1x > p2x)
      {
        result[0] = bottomLeftAx;
        result[1] = bottomLeftAy;
        clipPointAFound = true;
      }
      else
      {
        result[0] = topRightAx;
        result[1] = topLeftAy;
        clipPointAFound = true;
      }
    }
    else if (slopeA == slopePrime)
    {
      if (p1x > p2x)
      {
        result[0] = topLeftAx;
        result[1] = topLeftAy;
        clipPointAFound = true;
      }
      else
      {
        result[0] = bottomRightAx;
        result[1] = bottomLeftAy;
        clipPointAFound = true;
      }
    }

    //determine whether clipping point is the corner of nodeB
    if ((-slopeB) == slopePrime)
    {
      if (p2x > p1x)
      {
        result[2] = bottomLeftBx;
        result[3] = bottomLeftBy;
        clipPointBFound = true;
      }
      else
      {
        result[2] = topRightBx;
        result[3] = topLeftBy;
        clipPointBFound = true;
      }
    }
    else if (slopeB == slopePrime)
    {
      if (p2x > p1x)
      {
        result[2] = topLeftBx;
        result[3] = topLeftBy;
        clipPointBFound = true;
      }
      else
      {
        result[2] = bottomRightBx;
        result[3] = bottomLeftBy;
        clipPointBFound = true;
      }
    }

    //if both clipping points are corners
    if (clipPointAFound && clipPointBFound)
    {
      return false;
    }

    //determine Cardinal Direction of rectangles
    if (p1x > p2x)
    {
      if (p1y > p2y)
      {
        cardinalDirectionA = IGeometry.getCardinalDirection(slopeA, slopePrime, 4);
        cardinalDirectionB = IGeometry.getCardinalDirection(slopeB, slopePrime, 2);
      }
      else
      {
        cardinalDirectionA = IGeometry.getCardinalDirection(-slopeA, slopePrime, 3);
        cardinalDirectionB = IGeometry.getCardinalDirection(-slopeB, slopePrime, 1);
      }
    }
    else
    {
      if (p1y > p2y)
      {
        cardinalDirectionA = IGeometry.getCardinalDirection(-slopeA, slopePrime, 1);
        cardinalDirectionB = IGeometry.getCardinalDirection(-slopeB, slopePrime, 3);
      }
      else
      {
        cardinalDirectionA = IGeometry.getCardinalDirection(slopeA, slopePrime, 2);
        cardinalDirectionB = IGeometry.getCardinalDirection(slopeB, slopePrime, 4);
      }
    }
    //calculate clipping Point if it is not found before
    if (!clipPointAFound)
    {
      switch (cardinalDirectionA)
      {
        case 1:
          tempPointAy = topLeftAy;
          tempPointAx = p1x + (-halfHeightA) / slopePrime;
          result[0] = tempPointAx;
          result[1] = tempPointAy;
          break;
        case 2:
          tempPointAx = bottomRightAx;
          tempPointAy = p1y + halfWidthA * slopePrime;
          result[0] = tempPointAx;
          result[1] = tempPointAy;
          break;
        case 3:
          tempPointAy = bottomLeftAy;
          tempPointAx = p1x + halfHeightA / slopePrime;
          result[0] = tempPointAx;
          result[1] = tempPointAy;
          break;
        case 4:
          tempPointAx = bottomLeftAx;
          tempPointAy = p1y + (-halfWidthA) * slopePrime;
          result[0] = tempPointAx;
          result[1] = tempPointAy;
          break;
      }
    }
    if (!clipPointBFound)
    {
      switch (cardinalDirectionB)
      {
        case 1:
          tempPointBy = topLeftBy;
          tempPointBx = p2x + (-halfHeightB) / slopePrime;
          result[2] = tempPointBx;
          result[3] = tempPointBy;
          break;
        case 2:
          tempPointBx = bottomRightBx;
          tempPointBy = p2y + halfWidthB * slopePrime;
          result[2] = tempPointBx;
          result[3] = tempPointBy;
          break;
        case 3:
          tempPointBy = bottomLeftBy;
          tempPointBx = p2x + halfHeightB / slopePrime;
          result[2] = tempPointBx;
          result[3] = tempPointBy;
          break;
        case 4:
          tempPointBx = bottomLeftBx;
          tempPointBy = p2y + (-halfWidthB) * slopePrime;
          result[2] = tempPointBx;
          result[3] = tempPointBy;
          break;
      }
    }
  }
  return false;
}

IGeometry.getCardinalDirection = function (slope, slopePrime, line)
{
  if (slope > slopePrime)
  {
    return line;
  }
  else
  {
    return 1 + line % 4;
  }
}

IGeometry.getIntersection = function (s1, s2, f1, f2)
{
  if (f2 == null) {
    return IGeometry.getIntersection2(s1, s2, f1);
  }
  var x1 = s1.x;
  var y1 = s1.y;
  var x2 = s2.x;
  var y2 = s2.y;
  var x3 = f1.x;
  var y3 = f1.y;
  var x4 = f2.x;
  var y4 = f2.y;
  var x, y; // intersection point
  var a1, a2, b1, b2, c1, c2; // coefficients of line eqns.
  var denom;

  a1 = y2 - y1;
  b1 = x1 - x2;
  c1 = x2 * y1 - x1 * y2;  // { a1*x + b1*y + c1 = 0 is line 1 }

  a2 = y4 - y3;
  b2 = x3 - x4;
  c2 = x4 * y3 - x3 * y4;  // { a2*x + b2*y + c2 = 0 is line 2 }

  denom = a1 * b2 - a2 * b1;

  if (denom == 0)
  {
    return null;
  }

  x = (b1 * c2 - b2 * c1) / denom;
  y = (a2 * c1 - a1 * c2) / denom;

  return new Point(x, y);
}

// -----------------------------------------------------------------------------
// Section: Class Constants
// -----------------------------------------------------------------------------
/**
 * Some useful pre-calculated constants
 */
IGeometry.HALF_PI = 0.5 * Math.PI;
IGeometry.ONE_AND_HALF_PI = 1.5 * Math.PI;
IGeometry.TWO_PI = 2.0 * Math.PI;
IGeometry.THREE_PI = 3.0 * Math.PI;

module.exports = IGeometry;

},{}],16:[function(_dereq_,module,exports){
function IMath() {
}

/**
 * This method returns the sign of the input value.
 */
IMath.sign = function (value) {
  if (value > 0)
  {
    return 1;
  }
  else if (value < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

IMath.floor = function (value) {
  return value < 0 ? Math.ceil(value) : Math.floor(value);
}

IMath.ceil = function (value) {
  return value < 0 ? Math.floor(value) : Math.ceil(value);
}

module.exports = IMath;

},{}],17:[function(_dereq_,module,exports){
function Integer() {
}

Integer.MAX_VALUE = 2147483647;
Integer.MIN_VALUE = -2147483648;

module.exports = Integer;

},{}],18:[function(_dereq_,module,exports){
var LGraphObject = _dereq_('./LGraphObject');

function LEdge(source, target, vEdge) {
  LGraphObject.call(this, vEdge);

  this.isOverlapingSourceAndTarget = false;
  this.vGraphObject = vEdge;
  this.bendpoints = [];
  this.source = source;
  this.target = target;
}

LEdge.prototype = Object.create(LGraphObject.prototype);

for (var prop in LGraphObject) {
  LEdge[prop] = LGraphObject[prop];
}

LEdge.prototype.getSource = function ()
{
  return this.source;
};

LEdge.prototype.getTarget = function ()
{
  return this.target;
};

LEdge.prototype.isInterGraph = function ()
{
  return this.isInterGraph;
};

LEdge.prototype.getLength = function ()
{
  return this.length;
};

LEdge.prototype.isOverlapingSourceAndTarget = function ()
{
  return this.isOverlapingSourceAndTarget;
};

LEdge.prototype.getBendpoints = function ()
{
  return this.bendpoints;
};

LEdge.prototype.getLca = function ()
{
  return this.lca;
};

LEdge.prototype.getSourceInLca = function ()
{
  return this.sourceInLca;
};

LEdge.prototype.getTargetInLca = function ()
{
  return this.targetInLca;
};

LEdge.prototype.getOtherEnd = function (node)
{
  if (this.source === node)
  {
    return this.target;
  }
  else if (this.target === node)
  {
    return this.source;
  }
  else
  {
    throw "Node is not incident with this edge";
  }
}

LEdge.prototype.getOtherEndInGraph = function (node, graph)
{
  var otherEnd = this.getOtherEnd(node);
  var root = graph.getGraphManager().getRoot();

  while (true)
  {
    if (otherEnd.getOwner() == graph)
    {
      return otherEnd;
    }

    if (otherEnd.getOwner() == root)
    {
      break;
    }

    otherEnd = otherEnd.getOwner().getParent();
  }

  return null;
};

LEdge.prototype.updateLength = function ()
{
  var clipPointCoordinates = new Array(4);

  this.isOverlapingSourceAndTarget =
          IGeometry.getIntersection(this.target.getRect(),
                  this.source.getRect(),
                  clipPointCoordinates);

  if (!this.isOverlapingSourceAndTarget)
  {
    this.lengthX = clipPointCoordinates[0] - clipPointCoordinates[2];
    this.lengthY = clipPointCoordinates[1] - clipPointCoordinates[3];

    if (Math.abs(this.lengthX) < 1.0)
    {
      this.lengthX = IMath.sign(this.lengthX);
    }

    if (Math.abs(this.lengthY) < 1.0)
    {
      this.lengthY = IMath.sign(this.lengthY);
    }

    this.length = Math.sqrt(
            this.lengthX * this.lengthX + this.lengthY * this.lengthY);
  }
};

LEdge.prototype.updateLengthSimple = function ()
{
  this.lengthX = this.target.getCenterX() - this.source.getCenterX();
  this.lengthY = this.target.getCenterY() - this.source.getCenterY();

  if (Math.abs(this.lengthX) < 1.0)
  {
    this.lengthX = IMath.sign(this.lengthX);
  }

  if (Math.abs(this.lengthY) < 1.0)
  {
    this.lengthY = IMath.sign(this.lengthY);
  }

  this.length = Math.sqrt(
          this.lengthX * this.lengthX + this.lengthY * this.lengthY);
}

module.exports = LEdge;

},{"./LGraphObject":21}],19:[function(_dereq_,module,exports){
var LGraphObject = _dereq_('./LGraphObject');
var Integer = _dereq_('./Integer');
var LayoutConstants = _dereq_('./LayoutConstants');
var LGraphManager = _dereq_('./LGraphManager');
var LNode = _dereq_('./LNode');

function LGraph(parent, obj2, vGraph) {
  LGraphObject.call(this, vGraph);
  this.estimatedSize = Integer.MIN_VALUE;
  this.margin = LayoutConstants.DEFAULT_GRAPH_MARGIN;
  this.edges = [];
  this.nodes = [];
  this.isConnected = false;
  this.parent = parent;

  if (obj2 != null && obj2 instanceof LGraphManager) {
    this.graphManager = obj2;
  }
  else if (obj2 != null && obj2 instanceof Layout) {
    this.graphManager = obj2.graphManager;
  }
}

LGraph.prototype = Object.create(LGraphObject.prototype);
for (var prop in LGraphObject) {
  LGraph[prop] = LGraphObject[prop];
}

LGraph.prototype.getNodes = function () {
  return this.nodes;
};

LGraph.prototype.getEdges = function () {
  return this.edges;
};

LGraph.prototype.getGraphManager = function ()
{
  return this.graphManager;
};

LGraph.prototype.getParent = function ()
{
  return this.parent;
};

LGraph.prototype.getLeft = function ()
{
  return this.left;
};

LGraph.prototype.getRight = function ()
{
  return this.right;
};

LGraph.prototype.getTop = function ()
{
  return this.top;
};

LGraph.prototype.getBottom = function ()
{
  return this.bottom;
};

LGraph.prototype.isConnected = function ()
{
  return this.isConnected;
};

LGraph.prototype.add = function (obj1, sourceNode, targetNode) {
  if (sourceNode == null && targetNode == null) {
    var newNode = obj1;
    if (this.graphManager == null) {
      throw "Graph has no graph mgr!";
    }
    if (this.getNodes().indexOf(newNode) > -1) {
      throw "Node already in graph!";
    }
    newNode.owner = this;
    this.getNodes().push(newNode);

    return newNode;
  }
  else {
    var newEdge = obj1;
    if (!(this.getNodes().indexOf(sourceNode) > -1 && (this.getNodes().indexOf(targetNode)) > -1)) {
      throw "Source or target not in graph!";
    }

    if (!(sourceNode.owner == targetNode.owner && sourceNode.owner == this)) {
      throw "Both owners must be this graph!";
    }

    if (sourceNode.owner != targetNode.owner)
    {
      return null;
    }

    // set source and target
    newEdge.source = sourceNode;
    newEdge.target = targetNode;

    // set as intra-graph edge
    newEdge.isInterGraph = false;

    // add to graph edge list
    this.getEdges().push(newEdge);

    // add to incidency lists
    sourceNode.edges.push(newEdge);

    if (targetNode != sourceNode)
    {
      targetNode.edges.push(newEdge);
    }

    return newEdge;
  }
};

LGraph.prototype.remove = function (obj) {
  var node = obj;
  if (obj instanceof LNode) {
    if (node == null) {
      throw "Node is null!";
    }
    if (!(node.owner != null && node.owner == this)) {
      throw "Owner graph is invalid!";
    }
    if (this.graphManager == null) {
      throw "Owner graph manager is invalid!";
    }
    // remove incident edges first (make a copy to do it safely)
    var edgesToBeRemoved = node.edges.slice();
    var edge;
    var s = edgesToBeRemoved.length;
    for (var i = 0; i < s; i++)
    {
      edge = edgesToBeRemoved[i];

      if (edge.isInterGraph)
      {
        this.graphManager.remove(edge);
      }
      else
      {
        edge.source.owner.remove(edge);
      }
    }

    // now the node itself
    var index = this.nodes.indexOf(node);
    if (index == -1) {
      throw "Node not in owner node list!";
    }

    this.nodes.splice(index, 1);
  }
  else if (obj instanceof LEdge) {
    var edge = obj;
    if (edge == null) {
      throw "Edge is null!";
    }
    if (!(edge.source != null && edge.target != null)) {
      throw "Source and/or target is null!";
    }
    if (!(edge.source.owner != null && edge.target.owner != null &&
            edge.source.owner == this && edge.target.owner == this)) {
      throw "Source and/or target owner is invalid!";
    }

    var sourceIndex = edge.source.edges.indexOf(edge);
    var targetIndex = edge.target.edges.indexOf(edge);
    if (!(sourceIndex > -1 && targetIndex > -1)) {
      throw "Source and/or target doesn't know this edge!";
    }

    edge.source.edges.splice(sourceIndex, 1);

    if (edge.target != edge.source)
    {
      edge.target.edges.splice(targetIndex, 1);
    }

    var index = edge.source.owner.getEdges().indexOf(edge);
    if (index == -1) {
      throw "Not in owner's edge list!";
    }

    edge.source.owner.getEdges().splice(index, 1);
  }
};

LGraph.prototype.updateLeftTop = function ()
{
  var top = Integer.MAX_VALUE;
  var left = Integer.MAX_VALUE;
  var nodeTop;
  var nodeLeft;

  var nodes = this.getNodes();
  var s = nodes.length;

  for (var i = 0; i < s; i++)
  {
    var lNode = nodes[i];
    nodeTop = Math.floor(lNode.getTop());
    nodeLeft = Math.floor(lNode.getLeft());

    if (top > nodeTop)
    {
      top = nodeTop;
    }

    if (left > nodeLeft)
    {
      left = nodeLeft;
    }
  }

  // Do we have any nodes in this graph?
  if (top == Integer.MAX_VALUE)
  {
    return null;
  }

  this.left = left - this.margin;
  this.top = top - this.margin;

  // Apply the margins and return the result
  return new Point(this.left, this.top);
};

LGraph.prototype.updateBounds = function (recursive)
{
  // calculate bounds
  var left = Integer.MAX_VALUE;
  var right = -Integer.MAX_VALUE;
  var top = Integer.MAX_VALUE;
  var bottom = -Integer.MAX_VALUE;
  var nodeLeft;
  var nodeRight;
  var nodeTop;
  var nodeBottom;

  var nodes = this.nodes;
  var s = nodes.length;
  for (var i = 0; i < s; i++)
  {
    var lNode = nodes[i];

    if (recursive && lNode.child != null)
    {
      lNode.updateBounds();
    }
    nodeLeft = Math.floor(lNode.getLeft());
    nodeRight = Math.floor(lNode.getRight());
    nodeTop = Math.floor(lNode.getTop());
    nodeBottom = Math.floor(lNode.getBottom());

    if (left > nodeLeft)
    {
      left = nodeLeft;
    }

    if (right < nodeRight)
    {
      right = nodeRight;
    }

    if (top > nodeTop)
    {
      top = nodeTop;
    }

    if (bottom < nodeBottom)
    {
      bottom = nodeBottom;
    }
  }

  var boundingRect = new RectangleD(left, top, right - left, bottom - top);
  if (left == Integer.MAX_VALUE)
  {
    this.left = Math.floor(this.parent.getLeft());
    this.right = Math.floor(this.parent.getRight());
    this.top = Math.floor(this.parent.getTop());
    this.bottom = Math.floor(this.parent.getBottom());
  }

  this.left = boundingRect.x - this.margin;
  this.right = boundingRect.x + boundingRect.width + this.margin;
  this.top = boundingRect.y - this.margin;
  this.bottom = boundingRect.y + boundingRect.height + this.margin;
};

LGraph.calculateBounds = function (nodes)
{
  var left = Integer.MAX_VALUE;
  var right = -Integer.MAX_VALUE;
  var top = Integer.MAX_VALUE;
  var bottom = -Integer.MAX_VALUE;
  var nodeLeft;
  var nodeRight;
  var nodeTop;
  var nodeBottom;

  var s = nodes.length;

  for (var i = 0; i < s; i++)
  {
    var lNode = nodes[i];
    nodeLeft = Math.floor(lNode.getLeft());
    nodeRight = Math.floor(lNode.getRight());
    nodeTop = Math.floor(lNode.getTop());
    nodeBottom = Math.floor(lNode.getBottom());

    if (left > nodeLeft)
    {
      left = nodeLeft;
    }

    if (right < nodeRight)
    {
      right = nodeRight;
    }

    if (top > nodeTop)
    {
      top = nodeTop;
    }

    if (bottom < nodeBottom)
    {
      bottom = nodeBottom;
    }
  }

  var boundingRect = new RectangleD(left, top, right - left, bottom - top);

  return boundingRect;
};

LGraph.prototype.getInclusionTreeDepth = function ()
{
  if (this == this.graphManager.getRoot())
  {
    return 1;
  }
  else
  {
    return this.parent.getInclusionTreeDepth();
  }
};

LGraph.prototype.getEstimatedSize = function ()
{
  if (this.estimatedSize == Integer.MIN_VALUE) {
    throw "assert failed";
  }
  return this.estimatedSize;
};

LGraph.prototype.calcEstimatedSize = function ()
{
  var size = 0;
  var nodes = this.nodes;
  var s = nodes.length;

  for (var i = 0; i < s; i++)
  {
    var lNode = nodes[i];
    size += lNode.calcEstimatedSize();
  }

  if (size == 0)
  {
    this.estimatedSize = LayoutConstants.EMPTY_COMPOUND_NODE_SIZE;
  }
  else
  {
    this.estimatedSize = Math.floor(size / Math.sqrt(this.nodes.length));
  }

  return Math.floor(this.estimatedSize);
};

LGraph.prototype.updateConnected = function ()
{
  if (this.nodes.length == 0)
  {
    this.isConnected = true;
    return;
  }

  var toBeVisited = [];
  var visited = new HashSet();
  var currentNode = this.nodes[0];
  var neighborEdges;
  var currentNeighbor;
  toBeVisited = toBeVisited.concat(currentNode.withChildren());

  while (toBeVisited.length > 0)
  {
    currentNode = toBeVisited.shift();
    visited.add(currentNode);

    // Traverse all neighbors of this node
    neighborEdges = currentNode.getEdges();
    var s = neighborEdges.length;
    for (var i = 0; i < s; i++)
    {
      var neighborEdge = neighborEdges[i];
      currentNeighbor =
              neighborEdge.getOtherEndInGraph(currentNode, this);

      // Add unvisited neighbors to the list to visit
      if (currentNeighbor != null &&
              !visited.contains(currentNeighbor))
      {
        toBeVisited = toBeVisited.concat(currentNeighbor.withChildren());
      }
    }
  }

  this.isConnected = false;

  if (visited.size() >= this.nodes.length)
  {
    var noOfVisitedInThisGraph = 0;

    var s = visited.size();
    for (var visitedId in visited.set)
    {
      var visitedNode = visited.set[visitedId];
      if (visitedNode.owner == this)
      {
        noOfVisitedInThisGraph++;
      }
    }

    if (noOfVisitedInThisGraph == this.nodes.length)
    {
      this.isConnected = true;
    }
  }
};

module.exports = LGraph;

},{"./Integer":17,"./LGraphManager":20,"./LGraphObject":21,"./LNode":22,"./LayoutConstants":24}],20:[function(_dereq_,module,exports){
function LGraphManager(layout) {
  this.layout = layout;

  this.graphs = [];
  this.edges = [];
}

LGraphManager.prototype.addRoot = function ()
{
  var ngraph = this.layout.newGraph();
  var nnode = this.layout.newNode(null);
  var root = this.add(ngraph, nnode);
  this.setRootGraph(root);
  return this.rootGraph;
};

LGraphManager.prototype.add = function (newGraph, parentNode, newEdge, sourceNode, targetNode)
{
  //there are just 2 parameters are passed then it adds an LGraph else it adds an LEdge
  if (newEdge == null && sourceNode == null && targetNode == null) {
    if (newGraph == null) {
      throw "Graph is null!";
    }
    if (parentNode == null) {
      throw "Parent node is null!";
    }
    if (this.graphs.indexOf(newGraph) > -1) {
      throw "Graph already in this graph mgr!";
    }

    this.graphs.push(newGraph);

    if (newGraph.parent != null) {
      throw "Already has a parent!";
    }
    if (parentNode.child != null) {
      throw  "Already has a child!";
    }

    newGraph.parent = parentNode;
    parentNode.child = newGraph;

    return newGraph;
  }
  else {
    //change the order of the parameters
    targetNode = newEdge;
    sourceNode = parentNode;
    newEdge = newGraph;
    var sourceGraph = sourceNode.getOwner();
    var targetGraph = targetNode.getOwner();

    if (!(sourceGraph != null && sourceGraph.getGraphManager() == this)) {
      throw "Source not in this graph mgr!";
    }
    if (!(targetGraph != null && targetGraph.getGraphManager() == this)) {
      throw "Target not in this graph mgr!";
    }

    if (sourceGraph == targetGraph)
    {
      newEdge.isInterGraph = false;
      return sourceGraph.add(newEdge, sourceNode, targetNode);
    }
    else
    {
      newEdge.isInterGraph = true;

      // set source and target
      newEdge.source = sourceNode;
      newEdge.target = targetNode;

      // add edge to inter-graph edge list
      if (this.edges.indexOf(newEdge) > -1) {
        throw "Edge already in inter-graph edge list!";
      }

      this.edges.push(newEdge);

      // add edge to source and target incidency lists
      if (!(newEdge.source != null && newEdge.target != null)) {
        throw "Edge source and/or target is null!";
      }

      if (!(newEdge.source.edges.indexOf(newEdge) == -1 && newEdge.target.edges.indexOf(newEdge) == -1)) {
        throw "Edge already in source and/or target incidency list!";
      }

      newEdge.source.edges.push(newEdge);
      newEdge.target.edges.push(newEdge);

      return newEdge;
    }
  }
};

LGraphManager.prototype.remove = function (lObj) {
  if (lObj instanceof LGraph) {
    var graph = lObj;
    if (graph.getGraphManager() != this) {
      throw "Graph not in this graph mgr";
    }
    if (!(graph == this.rootGraph || (graph.parent != null && graph.parent.graphManager == this))) {
      throw "Invalid parent node!";
    }

    // first the edges (make a copy to do it safely)
    var edgesToBeRemoved = [];

    edgesToBeRemoved = edgesToBeRemoved.concat(graph.getEdges());

    var edge;
    var s = edgesToBeRemoved.length;
    for (var i = 0; i < s; i++)
    {
      edge = edgesToBeRemoved[i];
      graph.remove(edge);
    }

    // then the nodes (make a copy to do it safely)
    var nodesToBeRemoved = [];

    nodesToBeRemoved = nodesToBeRemoved.concat(graph.getNodes());

    var node;
    s = nodesToBeRemoved.length;
    for (var i = 0; i < s; i++)
    {
      node = nodesToBeRemoved[i];
      graph.remove(node);
    }

    // check if graph is the root
    if (graph == this.rootGraph)
    {
      this.setRootGraph(null);
    }

    // now remove the graph itself
    var index = this.graphs.indexOf(graph);
    this.graphs.splice(index, 1);

    // also reset the parent of the graph
    graph.parent = null;
  }
  else if (lObj instanceof LEdge) {
    edge = lObj;
    if (edge == null) {
      throw "Edge is null!";
    }
    if (!edge.isInterGraph) {
      throw "Not an inter-graph edge!";
    }
    if (!(edge.source != null && edge.target != null)) {
      throw "Source and/or target is null!";
    }

    // remove edge from source and target nodes' incidency lists

    if (!(edge.source.edges.indexOf(edge) != -1 && edge.target.edges.indexOf(edge) != -1)) {
      throw "Source and/or target doesn't know this edge!";
    }

    var index = edge.source.edges.indexOf(edge);
    edge.source.edges.splice(index, 1);
    index = edge.target.edges.indexOf(edge);
    edge.target.edges.splice(index, 1);

    // remove edge from owner graph manager's inter-graph edge list

    if (!(edge.source.owner != null && edge.source.owner.getGraphManager() != null)) {
      throw "Edge owner graph or owner graph manager is null!";
    }
    if (edge.source.owner.getGraphManager().edges.indexOf(edge) == -1) {
      throw "Not in owner graph manager's edge list!";
    }

    var index = edge.source.owner.getGraphManager().edges.indexOf(edge);
    edge.source.owner.getGraphManager().edges.splice(index, 1);
  }
};

LGraphManager.prototype.updateBounds = function ()
{
  this.rootGraph.updateBounds(true);
};

LGraphManager.prototype.getGraphs = function ()
{
  return this.graphs;
};

LGraphManager.prototype.getAllNodes = function ()
{
  if (this.allNodes == null)
  {
    var nodeList = [];
    var graphs = this.getGraphs();
    var s = graphs.length;
    for (var i = 0; i < s; i++)
    {
      nodeList = nodeList.concat(graphs[i].getNodes());
    }
    this.allNodes = nodeList;
  }
  return this.allNodes;
};

LGraphManager.prototype.resetAllNodes = function ()
{
  this.allNodes = null;
};

LGraphManager.prototype.resetAllEdges = function ()
{
  this.allEdges = null;
};

LGraphManager.prototype.resetAllNodesToApplyGravitation = function ()
{
  this.allNodesToApplyGravitation = null;
};

LGraphManager.prototype.getAllEdges = function ()
{
  if (this.allEdges == null)
  {
    var edgeList = [];
    var graphs = this.getGraphs();
    var s = graphs.length;
    for (var i = 0; i < graphs.length; i++)
    {
      edgeList = edgeList.concat(graphs[i].getEdges());
    }

    edgeList = edgeList.concat(this.edges);

    this.allEdges = edgeList;
  }
  return this.allEdges;
};

LGraphManager.prototype.getAllNodesToApplyGravitation = function ()
{
  return this.allNodesToApplyGravitation;
};

LGraphManager.prototype.setAllNodesToApplyGravitation = function (nodeList)
{
  if (this.allNodesToApplyGravitation != null) {
    throw "assert failed";
  }

  this.allNodesToApplyGravitation = nodeList;
};

LGraphManager.prototype.getRoot = function ()
{
  return this.rootGraph;
};

LGraphManager.prototype.setRootGraph = function (graph)
{
  if (graph.getGraphManager() != this) {
    throw "Root not in this graph mgr!";
  }

  this.rootGraph = graph;
  // root graph must have a root node associated with it for convenience
  if (graph.parent == null)
  {
    graph.parent = this.layout.newNode("Root node");
  }
};

LGraphManager.prototype.getLayout = function ()
{
  return this.layout;
};

LGraphManager.prototype.isOneAncestorOfOther = function (firstNode, secondNode)
{
  if (!(firstNode != null && secondNode != null)) {
    throw "assert failed";
  }

  if (firstNode == secondNode)
  {
    return true;
  }
  // Is second node an ancestor of the first one?
  var ownerGraph = firstNode.getOwner();
  var parentNode;

  do
  {
    parentNode = ownerGraph.getParent();

    if (parentNode == null)
    {
      break;
    }

    if (parentNode == secondNode)
    {
      return true;
    }

    ownerGraph = parentNode.getOwner();
    if (ownerGraph == null)
    {
      break;
    }
  } while (true);
  // Is first node an ancestor of the second one?
  ownerGraph = secondNode.getOwner();

  do
  {
    parentNode = ownerGraph.getParent();

    if (parentNode == null)
    {
      break;
    }

    if (parentNode == firstNode)
    {
      return true;
    }

    ownerGraph = parentNode.getOwner();
    if (ownerGraph == null)
    {
      break;
    }
  } while (true);

  return false;
};

LGraphManager.prototype.calcLowestCommonAncestors = function ()
{
  var edge;
  var sourceNode;
  var targetNode;
  var sourceAncestorGraph;
  var targetAncestorGraph;

  var edges = this.getAllEdges();
  var s = edges.length;
  for (var i = 0; i < s; i++)
  {
    edge = edges[i];

    sourceNode = edge.source;
    targetNode = edge.target;
    edge.lca = null;
    edge.sourceInLca = sourceNode;
    edge.targetInLca = targetNode;

    if (sourceNode == targetNode)
    {
      edge.lca = sourceNode.getOwner();
      continue;
    }

    sourceAncestorGraph = sourceNode.getOwner();

    while (edge.lca == null)
    {
      targetAncestorGraph = targetNode.getOwner();

      while (edge.lca == null)
      {
        if (targetAncestorGraph == sourceAncestorGraph)
        {
          edge.lca = targetAncestorGraph;
          break;
        }

        if (targetAncestorGraph == this.rootGraph)
        {
          break;
        }

        if (edge.lca != null) {
          throw "assert failed";
        }
        edge.targetInLca = targetAncestorGraph.getParent();
        targetAncestorGraph = edge.targetInLca.getOwner();
      }

      if (sourceAncestorGraph == this.rootGraph)
      {
        break;
      }

      if (edge.lca == null)
      {
        edge.sourceInLca = sourceAncestorGraph.getParent();
        sourceAncestorGraph = edge.sourceInLca.getOwner();
      }
    }

    if (edge.lca == null) {
      throw "assert failed";
    }
  }
};

LGraphManager.prototype.calcLowestCommonAncestor = function (firstNode, secondNode)
{
  if (firstNode == secondNode)
  {
    return firstNode.getOwner();
  }
  var firstOwnerGraph = firstNode.getOwner();

  do
  {
    if (firstOwnerGraph == null)
    {
      break;
    }
    var secondOwnerGraph = secondNode.getOwner();

    do
    {
      if (secondOwnerGraph == null)
      {
        break;
      }

      if (secondOwnerGraph == firstOwnerGraph)
      {
        return secondOwnerGraph;
      }
      secondOwnerGraph = secondOwnerGraph.getParent().getOwner();
    } while (true);

    firstOwnerGraph = firstOwnerGraph.getParent().getOwner();
  } while (true);

  return firstOwnerGraph;
};

LGraphManager.prototype.calcInclusionTreeDepths = function (graph, depth) {
  if (graph == null && depth == null) {
    graph = this.rootGraph;
    depth = 1;
  }
  var node;

  var nodes = graph.getNodes();
  var s = nodes.length;
  for (var i = 0; i < s; i++)
  {
    node = nodes[i];
    node.inclusionTreeDepth = depth;

    if (node.child != null)
    {
      this.calcInclusionTreeDepths(node.child, depth + 1);
    }
  }
};

LGraphManager.prototype.includesInvalidEdge = function ()
{
  var edge;

  var s = this.edges.length;
  for (var i = 0; i < s; i++)
  {
    edge = this.edges[i];

    if (this.isOneAncestorOfOther(edge.source, edge.target))
    {
      return true;
    }
  }
  return false;
};

module.exports = LGraphManager;

},{}],21:[function(_dereq_,module,exports){
function LGraphObject(vGraphObject) {
  this.vGraphObject = vGraphObject;
}

module.exports = LGraphObject;

},{}],22:[function(_dereq_,module,exports){
var LGraphObject = _dereq_('./LGraphObject');
var Integer = _dereq_('./Integer');
var RectangleD = _dereq_('./RectangleD');

function LNode(gm, loc, size, vNode) {
  //Alternative constructor 1 : LNode(LGraphManager gm, Point loc, Dimension size, Object vNode)
  if (size == null && vNode == null) {
    vNode = loc;
  }

  LGraphObject.call(this, vNode);

  //Alternative constructor 2 : LNode(Layout layout, Object vNode)
  if (gm.graphManager != null)
    gm = gm.graphManager;

  this.estimatedSize = Integer.MIN_VALUE;
  this.inclusionTreeDepth = Integer.MAX_VALUE;
  this.vGraphObject = vNode;
  this.edges = [];
  this.graphManager = gm;

  if (size != null && loc != null)
    this.rect = new RectangleD(loc.x, loc.y, size.width, size.height);
  else
    this.rect = new RectangleD();
}

LNode.prototype = Object.create(LGraphObject.prototype);
for (var prop in LGraphObject) {
  LNode[prop] = LGraphObject[prop];
}

LNode.prototype.getEdges = function ()
{
  return this.edges;
};

LNode.prototype.getChild = function ()
{
  return this.child;
};

LNode.prototype.getOwner = function ()
{
  if (this.owner != null) {
    if (!(this.owner == null || this.owner.getNodes().indexOf(this) > -1)) {
      throw "assert failed";
    }
  }

  return this.owner;
};

LNode.prototype.getWidth = function ()
{
  return this.rect.width;
};

LNode.prototype.setWidth = function (width)
{
  this.rect.width = width;
};

LNode.prototype.getHeight = function ()
{
  return this.rect.height;
};

LNode.prototype.setHeight = function (height)
{
  this.rect.height = height;
};

LNode.prototype.getCenterX = function ()
{
  return this.rect.x + this.rect.width / 2;
};

LNode.prototype.getCenterY = function ()
{
  return this.rect.y + this.rect.height / 2;
};

LNode.prototype.getCenter = function ()
{
  return new PointD(this.rect.x + this.rect.width / 2,
          this.rect.y + this.rect.height / 2);
};

LNode.prototype.getLocation = function ()
{
  return new PointD(this.rect.x, this.rect.y);
};

LNode.prototype.getRect = function ()
{
  return this.rect;
};

LNode.prototype.getDiagonal = function ()
{
  return Math.sqrt(this.rect.width * this.rect.width +
          this.rect.height * this.rect.height);
};

LNode.prototype.setRect = function (upperLeft, dimension)
{
  this.rect.x = upperLeft.x;
  this.rect.y = upperLeft.y;
  this.rect.width = dimension.width;
  this.rect.height = dimension.height;
};

LNode.prototype.setCenter = function (cx, cy)
{
  this.rect.x = cx - this.rect.width / 2;
  this.rect.y = cy - this.rect.height / 2;
};

LNode.prototype.setLocation = function (x, y)
{
  this.rect.x = x;
  this.rect.y = y;
};

LNode.prototype.moveBy = function (dx, dy)
{
  this.rect.x += dx;
  this.rect.y += dy;
};

LNode.prototype.getEdgeListToNode = function (to)
{
  var edgeList = [];
  var edge;

  for (var obj in this.edges)
  {
    edge = obj;

    if (edge.target == to)
    {
      if (edge.source != this)
        throw "Incorrect edge source!";

      edgeList.push(edge);
    }
  }

  return edgeList;
};

LNode.prototype.getEdgesBetween = function (other)
{
  var edgeList = [];
  var edge;

  for (var obj in this.edges)
  {
    edge = this.edges[obj];

    if (!(edge.source == this || edge.target == this))
      throw "Incorrect edge source and/or target";

    if ((edge.target == other) || (edge.source == other))
    {
      edgeList.push(edge);
    }
  }

  return edgeList;
};

LNode.prototype.getNeighborsList = function ()
{
  var neighbors = new HashSet();
  var edge;

  for (var obj in this.edges)
  {
    edge = this.edges[obj];

    if (edge.source == this)
    {
      neighbors.add(edge.target);
    }
    else
    {
      if (!edge.target == this)
        throw "Incorrect incidency!";
      neighbors.add(edge.source);
    }
  }

  return neighbors;
};

LNode.prototype.withChildren = function ()
{
  var withNeighborsList = [];
  var childNode;

  withNeighborsList.push(this);

  if (this.child != null)
  {
    var nodes = this.child.getNodes();
    for (var i = 0; i < nodes.length; i++)
    {
      childNode = nodes[i];

      withNeighborsList = withNeighborsList.concat(childNode.withChildren());
    }
  }

  return withNeighborsList;
};

LNode.prototype.getEstimatedSize = function () {
  if (this.estimatedSize == Integer.MIN_VALUE) {
    throw "assert failed";
  }
  return this.estimatedSize;
};

LNode.prototype.calcEstimatedSize = function () {
  if (this.child == null)
  {
    return this.estimatedSize = Math.floor((this.rect.width + this.rect.height) / 2);
  }
  else
  {
    this.estimatedSize = this.child.calcEstimatedSize();
    this.rect.width = this.estimatedSize;
    this.rect.height = this.estimatedSize;

    return this.estimatedSize;
  }
};

LNode.prototype.scatter = function () {
  var randomCenterX;
  var randomCenterY;

  var minX = -LayoutConstants.INITIAL_WORLD_BOUNDARY;
  var maxX = LayoutConstants.INITIAL_WORLD_BOUNDARY;
  randomCenterX = LayoutConstants.WORLD_CENTER_X +
          (RandomSeed.nextDouble() * (maxX - minX)) + minX;

  var minY = -LayoutConstants.INITIAL_WORLD_BOUNDARY;
  var maxY = LayoutConstants.INITIAL_WORLD_BOUNDARY;
  randomCenterY = LayoutConstants.WORLD_CENTER_Y +
          (RandomSeed.nextDouble() * (maxY - minY)) + minY;

  this.rect.x = randomCenterX;
  this.rect.y = randomCenterY
};

LNode.prototype.updateBounds = function () {
  if (this.getChild() == null) {
    throw "assert failed";
  }
  if (this.getChild().getNodes().length != 0)
  {
    // wrap the children nodes by re-arranging the boundaries
    var childGraph = this.getChild();
    childGraph.updateBounds(true);

    this.rect.x = childGraph.getLeft();
    this.rect.y = childGraph.getTop();

    this.setWidth(childGraph.getRight() - childGraph.getLeft() +
            2 * LayoutConstants.COMPOUND_NODE_MARGIN);
    this.setHeight(childGraph.getBottom() - childGraph.getTop() +
            2 * LayoutConstants.COMPOUND_NODE_MARGIN +
            LayoutConstants.LABEL_HEIGHT);
  }
};

LNode.prototype.getInclusionTreeDepth = function ()
{
  if (this.inclusionTreeDepth == Integer.MAX_VALUE) {
    throw "assert failed";
  }
  return this.inclusionTreeDepth;
};

LNode.prototype.transform = function (trans)
{
  var left = this.rect.x;

  if (left > LayoutConstants.WORLD_BOUNDARY)
  {
    left = LayoutConstants.WORLD_BOUNDARY;
  }
  else if (left < -LayoutConstants.WORLD_BOUNDARY)
  {
    left = -LayoutConstants.WORLD_BOUNDARY;
  }

  var top = this.rect.y;

  if (top > LayoutConstants.WORLD_BOUNDARY)
  {
    top = LayoutConstants.WORLD_BOUNDARY;
  }
  else if (top < -LayoutConstants.WORLD_BOUNDARY)
  {
    top = -LayoutConstants.WORLD_BOUNDARY;
  }

  var leftTop = new PointD(left, top);
  var vLeftTop = trans.inverseTransformPoint(leftTop);

  this.setLocation(vLeftTop.x, vLeftTop.y);
};

LNode.prototype.getLeft = function ()
{
  return this.rect.x;
};

LNode.prototype.getRight = function ()
{
  return this.rect.x + this.rect.width;
};

LNode.prototype.getTop = function ()
{
  return this.rect.y;
};

LNode.prototype.getBottom = function ()
{
  return this.rect.y + this.rect.height;
};

LNode.prototype.getParent = function ()
{
  if (this.owner == null)
  {
    return null;
  }

  return this.owner.getParent();
};

module.exports = LNode;

},{"./Integer":17,"./LGraphObject":21,"./RectangleD":34}],23:[function(_dereq_,module,exports){
var LayoutConstants = _dereq_('./LayoutConstants');
var HashMap = _dereq_('./HashMap');
var LGraphManager = _dereq_('./LGraphManager');

function Layout(isRemoteUse) {
  //Layout Quality: 0:proof, 1:default, 2:draft
  this.layoutQuality = LayoutConstants.DEFAULT_QUALITY;
  //Whether layout should create bendpoints as needed or not
  this.createBendsAsNeeded =
          LayoutConstants.DEFAULT_CREATE_BENDS_AS_NEEDED;
  //Whether layout should be incremental or not
  this.incremental = LayoutConstants.DEFAULT_INCREMENTAL;
  //Whether we animate from before to after layout node positions
  this.animationOnLayout =
          LayoutConstants.DEFAULT_ANIMATION_ON_LAYOUT;
  //Whether we animate the layout process or not
  this.animationDuringLayout = LayoutConstants.DEFAULT_ANIMATION_DURING_LAYOUT;
  //Number iterations that should be done between two successive animations
  this.animationPeriod = LayoutConstants.DEFAULT_ANIMATION_PERIOD;
  /**
   * Whether or not leaf nodes (non-compound nodes) are of uniform sizes. When
   * they are, both spring and repulsion forces between two leaf nodes can be
   * calculated without the expensive clipping point calculations, resulting
   * in major speed-up.
   */
  this.uniformLeafNodeSizes =
          LayoutConstants.DEFAULT_UNIFORM_LEAF_NODE_SIZES;
  /**
   * This is used for creation of bendpoints by using dummy nodes and edges.
   * Maps an LEdge to its dummy bendpoint path.
   */
  this.edgeToDummyNodes = new HashMap();
  this.graphManager = new LGraphManager(this);
  this.isLayoutFinished = false;
  this.isSubLayout = false;
  this.isRemoteUse = false;

  if (isRemoteUse != null) {
    this.isRemoteUse = isRemoteUse;
  }
}

Layout.RANDOM_SEED = 1;

Layout.prototype.getGraphManager = function () {
  return this.graphManager;
};

Layout.prototype.getAllNodes = function () {
  return this.graphManager.getAllNodes();
};

Layout.prototype.getAllEdges = function () {
  return this.graphManager.getAllEdges();
};

Layout.prototype.getAllNodesToApplyGravitation = function () {
  return this.graphManager.getAllNodesToApplyGravitation();
};

Layout.prototype.newGraphManager = function () {
  var gm = new LGraphManager(this);
  this.graphManager = gm;
  return gm;
};

Layout.prototype.newGraph = function (vGraph)
{
  return new LGraph(null, this.graphManager, vGraph);
};

Layout.prototype.newNode = function (vNode)
{
  return new LNode(this.graphManager, vNode);
};

Layout.prototype.newEdge = function (vEdge)
{
  return new LEdge(null, null, vEdge);
};

Layout.prototype.runLayout = function ()
{
  this.isLayoutFinished = false;

  this.initParameters();
  var isLayoutSuccessfull;

  if ((this.graphManager.getRoot() == null)
          || this.graphManager.getRoot().getNodes().length == 0
          || this.graphManager.includesInvalidEdge())
  {
    isLayoutSuccessfull = false;
  }
  else
  {
    // calculate execution time
    var startTime = 0;

    if (!this.isSubLayout)
    {
      startTime = new Date().getTime()
    }

    isLayoutSuccessfull = this.layout();

    if (!this.isSubLayout)
    {
      var endTime = new Date().getTime();
      var excTime = endTime - startTime;

      console.log("Total execution time: " + excTime + " miliseconds.");
    }
  }

  if (isLayoutSuccessfull)
  {
    if (!this.isSubLayout)
    {
      this.doPostLayout();
    }
  }

  this.isLayoutFinished = true;

  return isLayoutSuccessfull;
};

/**
 * This method performs the operations required after layout.
 */
Layout.prototype.doPostLayout = function ()
{
  //assert !isSubLayout : "Should not be called on sub-layout!";
  // Propagate geometric changes to v-level objects
  this.transform();
  this.update();
};

/**
 * This method updates the geometry of the target graph according to
 * calculated layout.
 */
Layout.prototype.update2 = function () {
  // update bend points
  if (this.createBendsAsNeeded)
  {
    this.createBendpointsFromDummyNodes();

    // reset all edges, since the topology has changed
    this.graphManager.resetAllEdges();
  }

  // perform edge, node and root updates if layout is not called
  // remotely
  if (!this.isRemoteUse)
  {
    // update all edges
    var edge;
    var allEdges = this.graphManager.getAllEdges();
    for (var i = 0; i < allEdges.length; i++)
    {
      edge = allEdges[i];
//      this.update(edge);
    }

    // recursively update nodes
    var node;
    var nodes = this.graphManager.getRoot().getNodes();
    for (var i = 0; i < nodes.length; i++)
    {
      node = nodes[i];
//      this.update(node);
    }

    // update root graph
    this.update(this.graphManager.getRoot());
  }
};

Layout.prototype.update = function (obj) {
  if (obj == null) {
    this.update2();
  }
  else if (obj instanceof LNode) {
    var node = obj;
    if (node.getChild() != null)
    {
      // since node is compound, recursively update child nodes
      var nodes = node.getChild().getNodes();
      for (var i = 0; i < nodes.length; i++)
      {
        update(nodes[i]);
      }
    }

    // if the l-level node is associated with a v-level graph object,
    // then it is assumed that the v-level node implements the
    // interface Updatable.
    if (node.vGraphObject != null)
    {
      // cast to Updatable without any type check
      var vNode = node.vGraphObject;

      // call the update method of the interface
      vNode.update(node);
    }
  }
  else if (obj instanceof LEdge) {
    var edge = obj;
    // if the l-level edge is associated with a v-level graph object,
    // then it is assumed that the v-level edge implements the
    // interface Updatable.

    if (edge.vGraphObject != null)
    {
      // cast to Updatable without any type check
      var vEdge = edge.vGraphObject;

      // call the update method of the interface
      vEdge.update(edge);
    }
  }
  else if (obj instanceof LGraph) {
    var graph = obj;
    // if the l-level graph is associated with a v-level graph object,
    // then it is assumed that the v-level object implements the
    // interface Updatable.

    if (graph.vGraphObject != null)
    {
      // cast to Updatable without any type check
      var vGraph = graph.vGraphObject;

      // call the update method of the interface
      vGraph.update(graph);
    }
  }
};

/**
 * This method is used to set all layout parameters to default values
 * determined at compile time.
 */
Layout.prototype.initParameters = function () {
  if (!this.isSubLayout)
  {
    this.layoutQuality = LayoutConstants.DEFAULT_QUALITY;
    this.animationDuringLayout = LayoutConstants.DEFAULT_ANIMATION_ON_LAYOUT;
    this.animationPeriod = LayoutConstants.DEFAULT_ANIMATION_PERIOD;
    this.animationOnLayout = LayoutConstants.DEFAULT_ANIMATION_DURING_LAYOUT;
    this.incremental = LayoutConstants.DEFAULT_INCREMENTAL;
    this.createBendsAsNeeded = LayoutConstants.DEFAULT_CREATE_BENDS_AS_NEEDED;
    this.uniformLeafNodeSizes = LayoutConstants.DEFAULT_UNIFORM_LEAF_NODE_SIZES;
  }

  if (this.animationDuringLayout)
  {
    animationOnLayout = false;
  }
};

Layout.prototype.transform = function (newLeftTop) {
  if (newLeftTop == undefined) {
    this.transform(new PointD(0, 0));
  }
  else {
    // create a transformation object (from Eclipse to layout). When an
    // inverse transform is applied, we get upper-left coordinate of the
    // drawing or the root graph at given input coordinate (some margins
    // already included in calculation of left-top).

    var trans = new Transform();
    var leftTop = this.graphManager.getRoot().updateLeftTop();

    if (leftTop != null)
    {
      trans.setWorldOrgX(newLeftTop.x);
      trans.setWorldOrgY(newLeftTop.y);

      trans.setDeviceOrgX(leftTop.x);
      trans.setDeviceOrgY(leftTop.y);

      var nodes = this.getAllNodes();
      var node;

      for (var i = 0; i < nodes.length; i++)
      {
        node = nodes[i];
        node.transform(trans);
      }
    }
  }
};

Layout.prototype.positionNodesRandomly = function (graph) {

  if (graph == undefined) {
    //assert !this.incremental;
    this.positionNodesRandomly(this.getGraphManager().getRoot());
    this.getGraphManager().getRoot().updateBounds(true);
  }
  else {
    var lNode;
    var childGraph;

    var nodes = graph.getNodes();
    for (var i = 0; i < nodes.length; i++)
    {
      lNode = nodes[i];
      childGraph = lNode.getChild();

      if (childGraph == null)
      {
        lNode.scatter();
      }
      else if (childGraph.getNodes().length == 0)
      {
        lNode.scatter();
      }
      else
      {
        this.positionNodesRandomly(childGraph);
        lNode.updateBounds();
      }
    }
  }
};

/**
 * This method returns a list of trees where each tree is represented as a
 * list of l-nodes. The method returns a list of size 0 when:
 * - The graph is not flat or
 * - One of the component(s) of the graph is not a tree.
 */
Layout.prototype.getFlatForest = function ()
{
  var flatForest = [];
  var isForest = true;

  // Quick reference for all nodes in the graph manager associated with
  // this layout. The list should not be changed.
  var allNodes = this.graphManager.getRoot().getNodes();

  // First be sure that the graph is flat
  var isFlat = true;

  for (var i = 0; i < allNodes.length; i++)
  {
    if (allNodes[i].getChild() != null)
    {
      isFlat = false;
    }
  }

  // Return empty forest if the graph is not flat.
  if (!isFlat)
  {
    return flatForest;
  }

  // Run BFS for each component of the graph.

  var visited = new HashSet();
  var toBeVisited = [];
  var parents = new HashMap();
  var unProcessedNodes = [];

  unProcessedNodes = unProcessedNodes.concat(allNodes);

  // Each iteration of this loop finds a component of the graph and
  // decides whether it is a tree or not. If it is a tree, adds it to the
  // forest and continued with the next component.

  while (unProcessedNodes.length > 0 && isForest)
  {
    toBeVisited.push(unProcessedNodes[0]);

    // Start the BFS. Each iteration of this loop visits a node in a
    // BFS manner.
    while (toBeVisited.length > 0 && isForest)
    {
      //pool operation
      var currentNode = toBeVisited[0];
      toBeVisited.splice(0, 1);
      visited.add(currentNode);

      // Traverse all neighbors of this node
      var neighborEdges = currentNode.getEdges();

      for (var i = 0; i < neighborEdges.length; i++)
      {
        var currentNeighbor =
                neighborEdges[i].getOtherEnd(currentNode);

        // If BFS is not growing from this neighbor.
        if (parents.get(currentNode) != currentNeighbor)
        {
          // We haven't previously visited this neighbor.
          if (!visited.contains(currentNeighbor))
          {
            toBeVisited.push(currentNeighbor);
            parents.put(currentNeighbor, currentNode);
          }
          // Since we have previously visited this neighbor and
          // this neighbor is not parent of currentNode, given
          // graph contains a component that is not tree, hence
          // it is not a forest.
          else
          {
            isForest = false;
            break;
          }
        }
      }
    }

    // The graph contains a component that is not a tree. Empty
    // previously found trees. The method will end.
    if (!isForest)
    {
      flatForest = [];
    }
    // Save currently visited nodes as a tree in our forest. Reset
    // visited and parents lists. Continue with the next component of
    // the graph, if any.
    else
    {
      var temp = [];
      visited.addAllTo(temp);
      flatForest.push(temp);
      //flatForest = flatForest.concat(temp);
      //unProcessedNodes.removeAll(visited);
      for (var i = 0; i < temp.length; i++) {
        var value = temp[i];
        var index = unProcessedNodes.indexOf(value);
        if (index > -1) {
          unProcessedNodes.splice(index, 1);
        }
      }
      visited = new HashSet();
      parents = new HashMap();
    }
  }

  return flatForest;
};

/**
 * This method creates dummy nodes (an l-level node with minimal dimensions)
 * for the given edge (one per bendpoint). The existing l-level structure
 * is updated accordingly.
 */
Layout.prototype.createDummyNodesForBendpoints = function (edge)
{
  var dummyNodes = [];
  var prev = edge.source;

  var graph = this.graphManager.calcLowestCommonAncestor(edge.source, edge.target);

  for (var i = 0; i < edge.bendpoints.length; i++)
  {
    // create new dummy node
    var dummyNode = this.newNode(null);
    dummyNode.setRect(new Point(0, 0), new Dimension(1, 1));

    graph.add(dummyNode);

    // create new dummy edge between prev and dummy node
    var dummyEdge = this.newEdge(null);
    this.graphManager.add(dummyEdge, prev, dummyNode);

    dummyNodes.add(dummyNode);
    prev = dummyNode;
  }

  var dummyEdge = this.newEdge(null);
  this.graphManager.add(dummyEdge, prev, edge.target);

  this.edgeToDummyNodes.put(edge, dummyNodes);

  // remove real edge from graph manager if it is inter-graph
  if (edge.isInterGraph())
  {
    this.graphManager.remove(edge);
  }
  // else, remove the edge from the current graph
  else
  {
    graph.remove(edge);
  }

  return dummyNodes;
};

/**
 * This method creates bendpoints for edges from the dummy nodes
 * at l-level.
 */
Layout.prototype.createBendpointsFromDummyNodes = function ()
{
  var edges = [];
  edges = edges.concat(this.graphManager.getAllEdges());
  edges = this.edgeToDummyNodes.keySet().concat(edges);

  for (var k = 0; k < edges.length; k++)
  {
    var lEdge = edges[k];

    if (lEdge.bendpoints.length > 0)
    {
      var path = this.edgeToDummyNodes.get(lEdge);

      for (var i = 0; i < path.length; i++)
      {
        var dummyNode = path[i];
        var p = new PointD(dummyNode.getCenterX(),
                dummyNode.getCenterY());

        // update bendpoint's location according to dummy node
        var ebp = lEdge.bendpoints.get(i);
        ebp.x = p.x;
        ebp.y = p.y;

        // remove the dummy node, dummy edges incident with this
        // dummy node is also removed (within the remove method)
        dummyNode.getOwner().remove(dummyNode);
      }

      // add the real edge to graph
      this.graphManager.add(lEdge, lEdge.source, lEdge.target);
    }
  }
};

Layout.transform = function (sliderValue, defaultValue, minDiv, maxMul) {
  if (minDiv != undefined && maxMul != undefined) {
    var value = defaultValue;

    if (sliderValue <= 50)
    {
      var minValue = defaultValue / minDiv;
      value -= ((defaultValue - minValue) / 50) * (50 - sliderValue);
    }
    else
    {
      var maxValue = defaultValue * maxMul;
      value += ((maxValue - defaultValue) / 50) * (sliderValue - 50);
    }

    return value;
  }
  else {
    var a, b;

    if (sliderValue <= 50)
    {
      a = 9.0 * defaultValue / 500.0;
      b = defaultValue / 10.0;
    }
    else
    {
      a = 9.0 * defaultValue / 50.0;
      b = -8 * defaultValue;
    }

    return (a * sliderValue + b);
  }
};

/**
 * This method finds and returns the center of the given nodes, assuming
 * that the given nodes form a tree in themselves.
 */
Layout.findCenterOfTree = function (nodes)
{
  var list = [];
  list = list.concat(nodes);

  var removedNodes = [];
  var remainingDegrees = new HashMap();
  var foundCenter = false;
  var centerNode = null;

  if (list.length == 1 || list.length == 2)
  {
    foundCenter = true;
    centerNode = list[0];
  }

  for (var i = 0; i < list.length; i++)
  {
    var node = list[i];
    var degree = node.getNeighborsList().size();
    remainingDegrees.put(node, node.getNeighborsList().size());

    if (degree == 1)
    {
      removedNodes.push(node);
    }
  }

  var tempList = [];
  tempList = tempList.concat(removedNodes);

  while (!foundCenter)
  {
    var tempList2 = [];
    tempList2 = tempList2.concat(tempList);
    tempList = [];

    for (var i = 0; i < list.length; i++)
    {
      var node = list[i];

      var index = list.indexOf(node);
      if (index >= 0) {
        list.splice(index, 1);
      }

      var neighbours = node.getNeighborsList();

      for (var j in neighbours.set)
      {
        var neighbour = neighbours.set[j];
        if (removedNodes.indexOf(neighbour) < 0)
        {
          var otherDegree = remainingDegrees.get(neighbour);
          var newDegree = otherDegree - 1;

          if (newDegree == 1)
          {
            tempList.push(neighbour);
          }

          remainingDegrees.put(neighbour, newDegree);
        }
      }
    }

    removedNodes = removedNodes.concat(tempList);

    if (list.length == 1 || list.length == 2)
    {
      foundCenter = true;
      centerNode = list[0];
    }
  }

  return centerNode;
};

/**
 * During the coarsening process, this layout may be referenced by two graph managers
 * this setter function grants access to change the currently being used graph manager
 */
Layout.prototype.setGraphManager = function (gm)
{
  this.graphManager = gm;
};

module.exports = Layout;

},{"./HashMap":13,"./LGraphManager":20,"./LayoutConstants":24}],24:[function(_dereq_,module,exports){
function LayoutConstants() {
}

/**
 * Layout Quality
 */
LayoutConstants.PROOF_QUALITY = 0;
LayoutConstants.DEFAULT_QUALITY = 1;
LayoutConstants.DRAFT_QUALITY = 2;

/**
 * Default parameters
 */
LayoutConstants.DEFAULT_CREATE_BENDS_AS_NEEDED = false;
//LayoutConstants.DEFAULT_INCREMENTAL = true;
LayoutConstants.DEFAULT_INCREMENTAL = false;
LayoutConstants.DEFAULT_ANIMATION_ON_LAYOUT = true;
LayoutConstants.DEFAULT_ANIMATION_DURING_LAYOUT = false;
LayoutConstants.DEFAULT_ANIMATION_PERIOD = 50;
LayoutConstants.DEFAULT_UNIFORM_LEAF_NODE_SIZES = false;

// -----------------------------------------------------------------------------
// Section: General other constants
// -----------------------------------------------------------------------------
/*
 * Margins of a graph to be applied on bouding rectangle of its contents. We
 * assume margins on all four sides to be uniform.
 */
LayoutConstants.DEFAULT_GRAPH_MARGIN = 10;

/*
 * The height of the label of a compound. We assume the label of a compound
 * node is placed at the bottom with a dynamic width same as the compound
 * itself.
 */
LayoutConstants.LABEL_HEIGHT = 20;

/*
 * Additional margins that we maintain as safety buffer for node-node
 * overlaps. Compound node labels as well as graph margins are handled
 * separately!
 */
LayoutConstants.COMPOUND_NODE_MARGIN = 5;

/*
 * Default dimension of a non-compound node.
 */
LayoutConstants.SIMPLE_NODE_SIZE = 40;

/*
 * Default dimension of a non-compound node.
 */
LayoutConstants.SIMPLE_NODE_HALF_SIZE = LayoutConstants.SIMPLE_NODE_SIZE / 2;

/*
 * Empty compound node size. When a compound node is empty, its both
 * dimensions should be of this value.
 */
LayoutConstants.EMPTY_COMPOUND_NODE_SIZE = 40;

/*
 * Minimum length that an edge should take during layout
 */
LayoutConstants.MIN_EDGE_LENGTH = 1;

/*
 * World boundaries that layout operates on
 */
LayoutConstants.WORLD_BOUNDARY = 1000000;

/*
 * World boundaries that random positioning can be performed with
 */
LayoutConstants.INITIAL_WORLD_BOUNDARY = LayoutConstants.WORLD_BOUNDARY / 1000;

/*
 * Coordinates of the world center
 */
LayoutConstants.WORLD_CENTER_X = 1200;
LayoutConstants.WORLD_CENTER_Y = 900;

module.exports = LayoutConstants;

},{}],25:[function(_dereq_,module,exports){
var Organization = _dereq_('./Organization');

function MemberPack(childGraph)
{
    var members = [] /*ArrayList<SbgnPDNode>*/;
    members.concat(childGraph.getNodes());
    
    var org = new Organization();

    this.layout();

    var nodes = [];

    for (var i=0; i<childGraph.getNodes().length; i++)
    {
        nodes[i] = childGraph.getNodes()[i];
    }
}

MemberPack.prototype.layout = function ()
{
    var compar = [] /*new ComparableNode[members.size()]*/;

    var i=0;
    for (var j=0; j<this.members.length; j++)
    {
        compar[i++] = members[j];
    }
    
    compar.sort(this.compareNodes);
    this.members = [];
    
    for (var j=0; j<compar.length; j++)
    {
        this.members.push(compar[j].getNode());
    }
    
    for (var j=0; j<this.members.length; j++)
    {
        this.org.insertNode(members[j]);
    }

    // Compaction c = new Compaction(
    // (ArrayList<SbgnPDNode>) members);
    // c.perform();

};

MemberPack.prototype.getWidth = function ()
{
    return this.org.getWidth();
};

MemberPack.prototype.getHeight = function ()
{
    return this.org.getHeight();
};

MemberPack.prototype.adjustLocations = function (x, y)
{
    this.org.adjustLocations(x, y);
};

MemberPack.prototype.getMembers = function ()
{
    return this.members;
};

MemberPack.prototype.compareNodes = function (nodeA, nodeB)
{
    var aSize = nodeA.getWidth() * nodeA.getHeight();
    var bSize = nodeB.getWidth() * nodeB.getHeight();
    
    if (aSize > bSize)
    {
        return 1;
    }
    else if (aSize < bSize)
    {
        return -1;
    }
    else
    {
        return 0;
    }
    return this.members;
};

module.exports = MemberPack;
},{"./Organization":26}],26:[function(_dereq_,module,exports){
var SbgnPDConstants = _dereq_('./SbgnPDConstants');

/**
* Creates a container whose width and height is only the margins
*/
function Organization()
{
    this.width = SbgnPDConstants.COMPLEX_MEM_MARGIN * 2;
    this.height = SbgnPDConstants.COMPLEX_MEM_MARGIN * 2 ;

    this.rowWidth = [] /*new ArrayList<Double>()*/;
    this.rowHeight = [] /*new ArrayList<Double>()*/;

    this.rows = [[]];/*new ArrayList<LinkedList<SbgnPDNode>>();*/
}

Organization.prototype.getWidth = function ()
{
    shiftToLastRow();
    return this.width;
};

Organization.prototype.getHeight = function ()
{
    return this.height;
};

/**
* Scans the rowWidth array list and returns the index of the row that has
* the minimum width.
*/
Organization.prototype.getShortestRowIndex = function ()
{
    var r = -1;
    var min = Number.MAX_VALUE;
    
    for (var i = 0; i < this.rows.length; i++)
    {
        if (this.rowWidth[i] < min)
        {
            r = i;
            min = this.rowWidth[i];
        }
    }
    return r;
};

/**
* Scans the rowWidth array list and returns the index of the row that has
* the maximum width.
*/
Organization.prototype.getLongestRowIndex = function ()
{
    var r = -1;
    var max = Number.MAX_VALUE;

    for (var i = 0; i < this.rows.length; i++)
    {
        if (this.rowWidth[i] > max)
        {
            r = i;
            max = this.rowWidth[i];
        }
    }
    return r;
};

Organization.prototype.insertNode = function (node)
{
    if (this.rows.length <= 0)
    {
        insertNodeToRow(node, 0);
    }
    else if (canAddHorizontal(node.getWidth(), node.getHeight()))
    {
        insertNodeToRow(node, getShortestRowIndex());
    }
    else
    {
        insertNodeToRow(node, this.rows.length);
    }
};

/**
* This method performs tiling. If a new row is needed, it creates the row
* and places the new node there. Otherwise, it places the node to the end
* of the given row.
* 
* @param node
* @param rowIndex
*/
Organization.prototype.insertNodeToRow = function (node, rowIndex)
{
    // Add new row if needed
    if (rowIndex === this.rows.length)
    {
        this.rows.push([]);

        this.rowWidth.push(SbgnPDConstants.COMPLEX_MIN_WIDTH);
        this.rowHeight.push(0.0);

        // TODO: assert rows.size() == rowWidth.size();
    }

    // Update row width
    var w = this.rowWidth[rowIndex] + node.getWidth();

    if (this.rows[rowIndex] > 0)
    {
        w += SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER;
    }
    
    this.rowWidth.set[rowIndex] = w;

    // Update complex width
    if (this.width < w)
    {
        this.width = w;
    }

    // Update height
    var h = node.getHeight();
    if (rowIndex > 0)
    {
        h += SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER;
    }
    
    var extraHeight = 0;
    if (h > this.rowHeight[rowIndex])
    {
        extraHeight = this.rowHeight[rowIndex];
        this.rowHeight[rowIndex] = h;
        extraHeight = this.rowHeight[rowIndex] - extraHeight;
    }

    this.height += extraHeight;

    // Insert node
    this.rows[rowIndex].push(node);
};

/**
* If moving the last node from the longest row and adding it to the last
* row makes the bounding box smaller, do it.
*/
Organization.prototype.shiftToLastRow = function ()
{
    var longest = this.getLongestRowIndex();
    var last = this.rowWidth.length - 1;
    var row = rows[longest]; /*LinkedList<SbgnPDNode>*/
    var node = row[(row.length-1)];

    var diff = node.getWidth() + SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER;

    if (this.width - this.rowWidth[(this.rowWidth.length-1)] > diff && 
        this.rowHeight[(this.rowHeight.length-1)] > node.getHeight())
    {
        row.pop();
        this.rows[this.rows.length].push(node);
        this.rowWidth[longest] = this.rowWidth[longest] - diff;
        this.rowWidth[last] = this.rowWidth[last] + diff;

        this.width = this.rowWidth[this.getLongestRowIndex()];

        // Update height of the organization
        var maxHeight = Number.MIN_VALUE;
        for (var i = 0; i < row.length; i++)
        {
            if (row[i].getHeight() > maxHeight)
            {
                maxHeight = row[i].getHeight();
            }
        }
        if (longest > 0)
        {
            maxHeight += SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER;
        }

        var prevTotal = this.rowHeight[longest] + this.rowHeight[last];

        this.rowHeight[longest] = maxHeight;
        if (this.rowHeight[last] < 
            node.getHeight() + SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER)
        {
            this.rowHeight[last] =
                    node.getHeight() + SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER;
        }

        var finalTotal = this.rowHeight[longest] + this.rowHeight[last];
        this.height += (finalTotal - prevTotal);

        this.shiftToLastRow();
    }
};

Organization.prototype.canAddHorizontal = function (extraWidth, extraHeight)
{
    var sri = this.getShortestRowIndex();

    if (sri < 0)
    {
        return true;
    }
    
    var min = this.rowWidth[sri];
    var hDiff = 0;
    if (this.rowHeight[sri] < extraHeight)
    {
        if (sri > 0)
        {
            hDiff = extraHeight + 
                    SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER - 
                    this.rowHeight[sri];
        }
    }
    if ((this.width - min) >= 
        (extraWidth + SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER))
    {
        return true;
    }

    return (this.height + hDiff) > 
           (min + extraWidth + SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER);
};

Organization.prototype.adjustLocations = function (x, y)
{
    x += SbgnPDConstants.COMPLEX_MEM_MARGIN;
    y += SbgnPDConstants.COMPLEX_MEM_MARGIN;

    var left = x;

    for (var i=0; i<this.rows.length; i++)
    {
        var row = this.rows[i];
        
        x = left;
        var maxHeight = 0;
        for (var j=0; j<row.length; j++)
        {
            var node = row[j];
            node.setLocation(x, y);

            x += (node.getWidth() + SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER);

            if (node.getHeight() > maxHeight)
            {
                maxHeight = node.getHeight();
            }
        }

        y += (maxHeight + SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER);
    }
};

module.exports = Organization;

},{"./SbgnPDConstants":35}],27:[function(_dereq_,module,exports){
/*
 *This class is the javascript implementation of the Point.java class in jdk
 */
function Point(x, y, p) {
  this.x = null;
  this.y = null;
  if (x == null && y == null && p == null) {
    this.x = 0;
    this.y = 0;
  }
  else if (typeof x == 'number' && typeof y == 'number' && p == null) {
    this.x = x;
    this.y = y;
  }
  else if (x.constructor.name == 'Point' && y == null && p == null) {
    p = x;
    this.x = p.x;
    this.y = p.y;
  }
}

Point.prototype.getX = function () {
  return this.x;
}

Point.prototype.getY = function () {
  return this.y;
}

Point.prototype.getLocation = function () {
  return new Point(this.x, this.y);
}

Point.prototype.setLocation = function (x, y, p) {
  if (x.constructor.name == 'Point' && y == null && p == null) {
    p = x;
    this.setLocation(p.x, p.y);
  }
  else if (typeof x == 'number' && typeof y == 'number' && p == null) {
    //if both parameters are integer just move (x,y) location
    if (parseInt(x) == x && parseInt(y) == y) {
      this.move(x, y);
    }
    else {
      this.x = Math.floor(x + 0.5);
      this.y = Math.floor(y + 0.5);
    }
  }
}

Point.prototype.move = function (x, y) {
  this.x = x;
  this.y = y;
}

Point.prototype.translate = function (dx, dy) {
  this.x += dx;
  this.y += dy;
}

Point.prototype.equals = function (obj) {
  if (obj.constructor.name == "Point") {
    var pt = obj;
    return (this.x == pt.x) && (this.y == pt.y);
  }
  return this == obj;
}

Point.prototype.toString = function () {
  return new Point().constructor.name + "[x=" + this.x + ",y=" + this.y + "]";
}

module.exports = Point;

},{}],28:[function(_dereq_,module,exports){
function PointD(x, y) {
  if (x == null && y == null) {
    this.x = 0;
    this.y = 0;
  } else {
    this.x = x;
    this.y = y;
  }
}

PointD.prototype.getX = function ()
{
  return this.x;
};

PointD.prototype.getY = function ()
{
  return this.y;
};

PointD.prototype.setX = function (x)
{
  this.x = x;
};

PointD.prototype.setY = function (y)
{
  this.y = y;
};

PointD.prototype.getDifference = function (pt)
{
  return new DimensionD(this.x - pt.x, this.y - pt.y);
};

PointD.prototype.getCopy = function ()
{
  return new PointD(this.x, this.y);
};

PointD.prototype.translate = function (dim)
{
  this.x += dim.width;
  this.y += dim.height;
  return this;
};

module.exports = PointD;

},{}],29:[function(_dereq_,module,exports){
function Polyomino() 
{
    // the number of cells
    this.l = 0;

    //the resulting placement coordinates
    this.x = 0;
    this.y = 0;

    this.label = "";

    // polyomino cells
    this.coord = [];
}

module.exports = Polyomino;
},{}],30:[function(_dereq_,module,exports){
var PolyominoQuickSort = _dereq_('./PolyominoQuickSort');
var Integer = _dereq_('./Integer');
var RectangleD = _dereq_('./RectangleD');

function PolyominoPacking()
{
    // Polyomino array
   this.polyominoes = [];

   // Bounding rectangles of the polyominoes
   this.rect = [];

   // The grid
   this.grid = [[]];

   // Center point of the grid
   this.gcx = 0;
   this.gcy = 0;

   // Grid size
   this.sizeX = 0;
   this.sizeY = 0;

   // The number of already placed polyominoes
   this.curmino = 0;

   // Stores the ordering of the polyominoes
   this.ind = [];

   // Random generator
   //TODO: Random Rgen;
}

/**
* This method performs polyomino packing.
*/
PolyominoPacking.prototype.pack = function (pm, pcount)
{
    this.polyominoes = pm;
    this.rect = [];

    // make the initial grid
    this.makeGrid(100, 100, 0);

    // make the random permutation of polyomino cells and
    // calculate the bounding rectangles.
    // TODO: Rgen = new Random(1);
    for (var k = 0; k < pcount; k++)
    {
        this.RandomizeMino(k);
    }
    
    // order the polyominoes in increasing size
    var key = [];
    this.ind = [];

    for (var i = 0; i < pcount; i++)
    {
        key[i] = -(this.rect[i].getMaxX() - this.rect[i].getMinX())
                        - (this.rect[i].getMaxY() - this.rect[i].getMinY());
    }
    var qsort = new PolyominoQuickSort();
    qsort.sort(pcount, key, this.ind);

    // place one by one starting from the largest
    for (this.curmino = 0; this.curmino < pcount; this.curmino++)
    {
        putMino(this.ind[this.curmino]);
    }
};

/**
* This creates the grid of given dimensions and fills it with the already
* placed polyominoes.
*/
PolyominoPacking.prototype.makeGrid = function (dimx, dimy, mN)
{
    var i;

    // allocate the grid
    /* TODO: We don't need this! 
    this.grid = [[]];
    for (i = 0; i < dimy; i++)
    {
        this.grid[i] = new byte[dimx];
    }*/

    var dx = dimx / 2 - this.gcx;
    var dy = dimy / 2 - this.gcy;
    this.gcx = dimx / 2;
    this.gcy = dimy / 2;
    this.sizeX = dimx;
    this.sizeY = dimy;

    // mark the positions occupied with the already placed
    // polyominoes.
    for (i = 0; i < mN; i++)
    {
        var p = this.polyominoes[this.ind[i]];
        p.x += dx;
        p.y += dy;

        for (var k = 0; k < p.l; k++)
        {
            var xx = p.coord[k].getX() + p.x;
            var yy = p.coord[k].getY() + p.y;
            this.grid[yy][xx] = 1;
        }
    }
};

/**
* This method checks whether p can be placed in (x,y). For each polyomino,
* check if the (x,y) is occupied/fits in the grid.
*/
PolyominoPacking.prototype.IsFreePlace = function (x, y, p)
{
    for (var k = 0; k < p.l; k++)
    {
        var xx = p.coord[k].getX() + x;
        var yy = p.coord[k].getY() + y;
        
        // return false if the polyomino goes outside the grid
        if (xx < 0 || yy < 0 || xx >= this.sizeX || yy >= this.sizeY)
        {
            return false;
        }
        
        // or the position is occupied
        if (grid[yy][xx] !== 0)
        {
            return false;
        }
    }

    // remember the position
    p.x = x;
    p.y = y;
    return true;
};

/**
* This tries to find a free place in the grid. The function returns true if
* the placement is successful.
*/
PolyominoPacking.prototype.tryPlacing = function (pi)
{
    var p = this.polyominoes[pi];

    var cx = this.gcx - (this.rect[pi].getMaxX() + this.rect[pi].getMinX()) / 2;
    var cy = this.gcy - (this.rect[pi].getMaxY() + this.rect[pi].getMinY()) / 2;

    // see if the center point is not occupied
    if (this.IsFreePlace(cx, cy, p))
    {
        return true;
    }

    // try placing in the increasing distance from the center
    for (var d = 1; d < this.sizeX / 2; d++)
    {
        for (var i = -d; i < d; i++)
        {
            var i1 = (i + d + 1) / 2 * (((i & 1) === 1) ? 1 : -1);
            if (this.IsFreePlace(-d + cx, -i1 + cy, p))
                    return true;
            if (this.IsFreePlace(d + cx, i1 + cy, p))
                    return true;
            if (this.IsFreePlace(cx - i1, d + cy, p))
                    return true;
            if (this.IsFreePlace(i1 + cx, -d + cy, p))
                    return true;
        }
    }
    return false;
};

/**
* This method places the given polyomino. The grid is enlarged if
* necessary.
*/
PolyominoPacking.prototype.putMino = function (pi)
{
    var p = this.polyominoes[pi];

    // if the polyomino cannot be placed in the current grid,
    // enlarge it.
    while (!this.tryPlacing(pi))
    {
        this.sizeX += 10;
        this.sizeY += 10;
        this.makeGrid(this.sizeX, this.sizeY, this.curmino);
    }

    // mark the positions occupied
    for (var k = 0; k < p.l; k++)
    {
        var xx = p.coord[k].getX() + p.x;
        var yy = p.coord[k].getY() + p.y;
        this.grid[yy][xx] = 1;
    }
};

/**
* This method makes a random permutation of polyomino cells and calculates
* the bounding rectangles of the polyominoes.
*/
PolyominoPacking.prototype.RandomizeMino = function (pi)
{
    var p = polyominoes[pi];
    var i;

    // make the random permutation. Theoretically it speeds up the
    // algorithm a little.
    for (i = 0; i < p.l; i++)
    {
        var i1 = Math.random() * (p.l - i) + i;
        var tmp = p.coord[i];
        p.coord[i] = p.coord[i1];
        p.coord[i1] = tmp;
    }

    // calculate the bounding rectangle of the polyomino
    this.rect[pi] = new RectangleD();

    var minX = Integer.MAX_VALUE;
    var minY = Integer.MAX_VALUE;
    var maxX = Integer.MIN_VALUE;
    var maxY = Integer.MIN_VALUE;
    p.x = p.y = 0;

    for (i = 0; i < p.l; i++)
    {
        if (p.coord[i].getX() < minX)
        {
            minX = p.coord[i].getX();
        }
        if (p.coord[i].getY() < minY)
        {
            minY = p.coord[i].getY();
        }
        if (p.coord[i].getX() > maxX)
        {
            maxX = p.coord[i].getX();
        }
        if (p.coord[i].getY() > maxY)
        {
            maxY = p.coord[i].getY();
        }
    }

    this.rect[pi].x = minX;
    this.rect[pi].y = minY;
    this.rect[pi].width = maxX - minX;
    this.rect[pi].height = maxY - minY;
};

module.exports = PolyominoPacking;
},{"./Integer":17,"./PolyominoQuickSort":31,"./RectangleD":34}],31:[function(_dereq_,module,exports){
function PolyominoQuickSort()
{
    // This variable stores the number of elements to be sorted
    this.n = 0;

    // This array stores the elements to be sorted
    this.key = [];

    // This array is used by static sorter and represents the current
    // order of the input array
    this.index = [];
}

/**
* This method is a static sorter.
*  Paramethers:
*      n - is the number of doubles to be sorted;
*      key - is array of doubles to be sorted. The order of elements
*          in this array is preserved;
*      index - in this array the sorted order of elements is
*          returned. index[i] = j means that i-th greatest element
*          is j-th element of array key.
*/
PolyominoQuickSort.prototype.sort = function (n, key, index)
{
   // store all paramethers as the instance variables
   this.n = n;
   this.key = key;
   this.index = index;

   // make the initial order of elements
   for (var i = 0; i < this.n; i++)
       this.index[i] = i;

   // call the recursive sorting method on the whole array
   this.recSortStatic(0, this.n-1);

   // do the final sort with the insertion sorting algorithm
   // because the recursive quicksort routine do not sort intervals
   // less than 10
   this.insertionSortStatic();
};

/**
* This method is a special sorter that sorts an integer array.
* The method is special because the elements a and b of the array
* are not compared as integer values. The elements of other array
* (array of keys) with indices a and b are compared instead. If two
* elements have equal keys, then the relative ordering of these
* elements is preserved.
*  Paramethers:
*      n - is the number of elements in the integer array;
*      index - this is the integer array to be sorted;
*      key - this array acts as keys for comparison of two integer
*              elements. It is assumed that elements of the array
*              index are in the range [0 .. key.length-1].
*/
PolyominoQuickSort.prototype.indexSort = function (n, index, key)
{
   // We want to reduce the problem to the simple static sorting.
   // First a mapping f from the set {0,..,n-1} to the set of
   // elements is defined by f(i) -> index[i]. Then the problem
   // simply reduces to the static sort where i-th key value is
   // key[f(i)].

   // allocate memory for the temporary arrays for the static sort
   var tempIndex = [];
   var tempKey = [];

   // this array will store the copy of array index
   var oldIndex = [];

   for (var i = 0; i < n; i++)
   {
       // store the key for static sort
       tempKey[i] = key[index[i]];

       // remember the i-th value of the index array
       oldIndex[i] = index[i];
   }

   // sort the auxiliary arrays
   this.sort(n, tempKey, tempIndex);

   // and put the result back in the index array

   for (var i = 0; i < n; i++)
       index[i] = oldIndex[tempIndex[i]];
};

/**
* This method is a non-static sorter.
*  Paramethers:
*      n - is the number of doubles to be sorted;
*      key - is array of doubles to be sorted.
*/
PolyominoQuickSort.prototype.sort = function (n, key)
{
   // store all paramethers as the instance variables
   this.n = n;
   this.key = key;

   // call the recursive sorting method on the whole array
   this.recSortNonStatic(0, this.n-1);

   // do the final sort with the insertion sorting algorithm
   // because the recursive quicksort routine do not sort intervals
   // less than 10
   this.insertionSortNonStatic();
};

//----------------------------------------------------------------------
// Section: Methods for static sorter
//----------------------------------------------------------------------

/**
 * This method do the recursive sorting with the quicksort
 * algorithm for static sorter. The interval to be sorted is
 * specified by the paramethers.
 */
PolyominoQuickSort.prototype.recSortStatic = function (left, right)
{
    // return immediately if the interval specified is too short
    if (left+10 > right)
    {
        return;
    }

    // choose the pivot and swap it with the last element of
    // the given interval
    var pivot = this.median3Static(left, right);

    // iterate through the given interval from both ends
    // simultaneously
    var i = left;
    var j = right-1;

    for (;;)
    {
        // increment i while i-th element is less than pivot
        do
        {
            i++;
        }
        while (this.cmp(this.index[i], pivot) === -1);

        // decrement j while j-th element is greater than pivot
        do
        {
            j--;
        }
        while (this.cmp(this.index[j], pivot) === 1);

        // if i and j cross then we are done, otherwise swap i-th
        // and j-th elements

        if (i < j)
        {
            this.swapStatic(i,j);
        }
        else
        {
            break;
        }
    }

    // place the pivot in the right place
    this.swapStatic(i, right-1);

    // recursively sort both parts to the left and to the right
    // from the pivot
    this.recSortStatic(left, i-1);
    this.recSortStatic(i+1, right);
};

/**
* This method chooses the middle element of left, right and center
* element of given interval and returns the index of this element.
* This method is used by static sorter.
*/
PolyominoQuickSort.prototype.median3Static = function (left, right)
{
   // calculates the center of the given interval
   var center = (left+right)/2;

   // if left element is greater than center element, swap them
   if (this.cmp(this.index[left], this.index[center]) === 1)
   {
       this.swapStatic(left,center);
   }
   
   // if left element is greater than right element, swap them
   if (this.cmp(this.index[left], this.index[right]) === 1)
   {
       this.swapStatic(left,right);
   }
   
   // if center element is greater than right element, swap them
   if (this.cmp(this.index[center], this.index[right]) === 1)
   {
       this.swapStatic(center,right);
   }
   
   // now the center element is less or equal than right element
   // and greater or equal than left element

   // move the center element to the right side by swaping it with
   // the one before the right element
   this.swapStatic(center, right-1);

   // return the pivot's index
   return this.index[right-1];
};

/**
* This method compares two elements specified with their indices.
* It returns -1, if i-th element is less than j-th element, and 1,
* if j-th element is less than i-th element. If both elements are
* equal then their indices i and j are compared. This method is
* used only by static sorter.
*/
PolyominoQuickSort.prototype.cmp = function (i, j)
{
   if (this.key[i] < this.key[j])
   {
       return -1;
   }
   
   if (this.key[i] > this.key[j])
   {
       return 1;
   }
   
   if (i < j)
   {
       return -1;
   }
   
   if (i > j)
   {
       return 1;
   }
   
   return 0;
};

/**
* This method swaps the elements by swapping their indices. Used
* only by static sorter.
*/
PolyominoQuickSort.prototype.swapStatic = function (i, j)
{
   var temp = this.index[i];
   this.index[i] = this.index[j];
   this.index[j] = temp;
};

/**
* This method sorts the array with the insertion sort algorithm.
* This method is used only by static sorted and therefore it works
* with array of indices instead of input array itself.
*/
PolyominoQuickSort.prototype.insertionSortStatic = function ()
{
   // iterate through all elements
   for (var i = 1; i < this.n; i++)
   {
       // stores the index of current element
       var temp = this.index[i];

       // search the right spot for current element by iterating
       // backwards and pushing greater elements one place up
       var j = i;

       while (j >= 1 && this.cmp(this.index[j-1], temp) === 1)
       {
           // j-th element is greater than current so move it up
           this.index[j] = this.index[j-1];
           j--;
       }

       // we found the new home for current element so store it
       this.index[j] = temp;
   }
};

//----------------------------------------------------------------------
// Section: Methods for non-static sorter
//----------------------------------------------------------------------

/**
 * This method do the recursive sorting with the quicksort
 * algorithm for non-static sorter. The interval to be sorted is
 * specified by the paramethers.
 */
PolyominoQuickSort.prototype.recSortNonStatic = function (left, right)
{
    // return immediately if the interval specified is too short
    if (left+10 > right)
    {
        return;
    }
    
    // choose the pivot and swap it with the last element of
    // the given interval
    var pivot = this.median3NonStatic(left, right);

    // iterate through the given interval from both ends
    // simultaneously
    var i = left;
    var j = right-1;

    for (;;)
    {
        // increment i while i-th element is less than pivot
        do
        {
            i++;
        }
        while (this.key[i] < pivot);

        // decrement j while j-th element is greater than pivot
        do
        {
            j--;
        }
        while (this.key[j] > pivot);

        // if i and j cross then we are done, otherwise swap i-th
        // and j-th elements
        if (i < j)
        {
            this.swapNonStatic(i, j);
        }
        else
        {
            break;
        }
    }

    // place the pivot in the right place
    this.swapNonStatic(i, right-1);

    // recursively sort both parts to the left and to the right
    // from the pivot
    this.recSortNonStatic(left, i-1);
    this.recSortNonStatic(i+1, right);
};

/**
* This method returns the middle element of left, right and center
* element. This method is used by non-static sorter.
*/
PolyominoQuickSort.prototype.median3NonStatic = function (left, right)
{
    // calculates the center of the given interval
    var center = (left+right)/2;

    // if left element is greater than center element, swap them
    if (this.key[left] > this.key[center])
    {
        this.swapNonStatic(left, center);
    }
   
    // if left element is greater than right element, swap them
    if (this.key[left] > this.key[right])
    {
        this.swapNonStatic(left, right);
    }
    
    // if center element is greater than right element, swap them
    if (this.key[center] > this.key[right])
    {
        this.swapNonStatic(center, right);
    }
    
    // now the center element is less or equal than right element
    // and greater or equal than left element

    // move the center element to the right side by swaping it with
    // the one before the right element
   this.swapNonStatic(center, right-1);

   // return the pivot
   return this.key[right-1];
};

/**
* This method swaps the elements. Used only by non-static sorter.
*/
PolyominoQuickSort.prototype.swapNonStatic = function (i, j)
{
   var temp = this.key[i];
   this.key[i] = this.key[j];
   this.key[j] = temp;
};

/**
* This method sorts the array with the insertion sort algorithm.
* This method is used only by non-static sorted.
*/
PolyominoQuickSort.prototype.insertionSortNonStatic = function ()
{
    // iterate through all elements
    for (var i = 1; i < this.n; i++)
    {
        // stores the current element
        var temp = this.key[i];

        // search the right spot for current element by iterating
        // backwards and pushing greater elements one place up
        var j = i;

        while (j >= 1 && this.key[j-1] > temp)
        {
            // j-1-th element is greater than current so move it up
            this.key[j] = this.key[j-1];
            j--;
        }
       
        // we found the new home for current element so store it
        this.key[j] = temp;
    }
};

module.exports = PolyominoQuickSort;
},{}],32:[function(_dereq_,module,exports){
function RandomSeed() {
}
RandomSeed.seed = 1;
RandomSeed.x = 0;

RandomSeed.nextDouble = function () {
  RandomSeed.x = Math.sin(RandomSeed.seed++) * 10000;
  return RandomSeed.x - Math.floor(RandomSeed.x);
};

module.exports = RandomSeed;

},{}],33:[function(_dereq_,module,exports){
var RandomSeed = _dereq_('./RandomSeed');
var Polyomino = _dereq_('./Polyomino');
var Point = _dereq_('./Point');

function RectProc() {
}

RectProc.AspectRatio = (1.0 / 1.0);// ysize/xsize

RectProc.PlaceRandomly = function (rN, rX1, rY1, rL, rH)
{
    var indexArray = [];

    var sumL = 0;
    var sumH = 0;

    for (var i = 0; i < rN; i++)
    {
        sumL += rL[i];
        sumH += rH[i];
        indexArray[i] = i;
    }
    
    for (var i = 0; i < rN; i++)
    {
        var a = RandomSeed.nextDouble(/* TODO: rN */);
        var tmp = indexArray[i];
        indexArray[i] = indexArray[a];
        indexArray[a] = tmp;
    }

    sumL /= rN;
    sumH /= rN;
    var numRows = (int) (Math.sqrt(rN) + 0.4999);

    for (var i = 0; i < rN; i++)
    {
        rX1[indexArray[i]] = (i / numRows) * sumL;
        rY1[indexArray[i]] = (i % numRows) * sumH;
    }
};

/**
* This method packs rectangles using polyomino packing algorithm.
* 
* @return
*/
RectProc.packRectanglesMino  = function (buffer, rN, rectangles)
{
    // make the intermediate data structure
    var rX1 = [];
    var rY1 = [];
    var rW = [];
    var rH = [];

    for (var i = 0; i < rN; i++)
    {
        rX1[i] = rectangles[i].getCenterX();
        rY1[i] = rectangles[i].getCenterY();
        rW[i] = rectangles[i].getWidth();
        rH[i] = rectangles[i].getHeight();
    }

    for (var i = 0; i < rN; i++)
    {
        rX1[i] -= rW[i] / 2;
        rY1[i] -= rH[i] / 2;
    }

    // do the packing
    RectProc.packRectanglesMino(buffer, rN, rX1, rW, rY1, rH, rectangles);

    // transfer back the results
    for (var i = 0; i < rN; i++)
    {
        rX1[i] += rW[i] / 2;
        rY1[i] += rH[i] / 2;
    }

    for (var i = 0; i < rN; i++)
    {
        rectangles[i].setCenter(rX1[i], rY1[i]);
    }
};

/**
* This method packs rectangles using polyomino packing algorithm.
*/

RectProc.packRectanglesMino = function (buffer, rN, rX, rW, rY, rH, rectangles)
{
    if (rN == 0)
    {
        return;
    }

    var stepX = 5;
    var stepY = 5;

    // dynamically calculate the grid step
    // double area = 0;
    //
    // for (int i = 0; i < rN; i++)
    // {
    // // stepX+=rL[i]+delta;
    // // stepY+=rH[i]+delta;
    // area += (rW[i] + delta) * (rH[i] + delta);
    // }
    //
    // double stepX = Math.sqrt(area / (rN * 16));
    // // (stepX+stepY)/(rN*8);
    //

    // adjust respecting the aspect ratio

    var fstep = 2 / (1 + RectProc.AspectRatio);
    stepY = stepX * RectProc.AspectRatio * fstep;
    stepX *= fstep;

    // make the polyomino representation
    var minos = [];

    for (var i = 0; i < rN; i++)
    {
        // size of the rectangle in grid units
        var W = Math.ceil((rW[i] + buffer) / stepX);
        var H = Math.ceil((rH[i] + buffer) / stepY);

        minos[i] = new Polyomino();
        minos[i].coord = [];

        // create the polyomino cells
        var cnt = 0;
        for (var y = 0; y < H; y++)
        {
            for (var x = 0; x < W; x++)
            {
                minos[i].coord[cnt] = new Point();
                minos[i].coord[cnt].x = x;
                minos[i].coord[cnt++].y = y;
            }
        }
        minos[i].l = cnt;
        minos[i].label = rectangles[i].label;
    }

    // do the packing
    var packer = new PolyominoPacking();
    packer.pack(minos, rN);

    // get the results
    for (var i = 0; i < rN; i++)
    {
        rX[i] = minos[i].x * stepX;
        rY[i] = minos[i].y * stepY;
    }
};

module.exports = RectProc;
},{"./Point":27,"./Polyomino":29,"./RandomSeed":32}],34:[function(_dereq_,module,exports){
function RectangleD(x, y, width, height) {
  this.x = 0;
  this.y = 0;
  this.width = 0;
  this.height = 0;

  if (x != null && y != null && width != null && height != null) {
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
  }
}

RectangleD.prototype.getX = function ()
{
  return this.x;
};

RectangleD.prototype.setX = function (x)
{
  this.x = x;
};

RectangleD.prototype.getY = function ()
{
  return this.y;
};

RectangleD.prototype.setY = function (y)
{
  this.y = y;
};

RectangleD.prototype.getWidth = function ()
{
  return this.width;
};

RectangleD.prototype.setWidth = function (width)
{
  this.width = width;
};

RectangleD.prototype.getHeight = function ()
{
  return this.height;
};

RectangleD.prototype.setHeight = function (height)
{
  this.height = height;
};

RectangleD.prototype.getRight = function ()
{
  return this.x + this.width;
};

RectangleD.prototype.getBottom = function ()
{
  return this.y + this.height;
};

RectangleD.prototype.intersects = function (a)
{
  if (this.getRight() < a.x)
  {
    return false;
  }

  if (this.getBottom() < a.y)
  {
    return false;
  }

  if (a.getRight() < this.x)
  {
    return false;
  }

  if (a.getBottom() < this.y)
  {
    return false;
  }

  return true;
};

RectangleD.prototype.getCenterX = function ()
{
  return this.x + this.width / 2;
};

RectangleD.prototype.getMinX = function ()
{
  return this.getX();
};

RectangleD.prototype.getMaxX = function ()
{
  return this.getX() + this.width;
};

RectangleD.prototype.getCenterY = function ()
{
  return this.y + this.height / 2;
};

RectangleD.prototype.getMinY = function ()
{
  return this.getY();
};

RectangleD.prototype.getMaxY = function ()
{
  return this.getY() + this.height;
};

RectangleD.prototype.getWidthHalf = function ()
{
  return this.width / 2;
};

RectangleD.prototype.getHeightHalf = function ()
{
  return this.height / 2;
};

module.exports = RectangleD;

},{}],35:[function(_dereq_,module,exports){
var CoSEConstants = _dereq_('./CoSEConstants');

function SbgnPDConstants() {
}

//SbgnPDConstants inherits static props in CoSEConstants 
for (var prop in CoSEConstants) {
  SbgnPDConstants[prop] = CoSEConstants[prop];
}

// Below are the SBGN glyph specific types.
SbgnPDConstants.MACROMOLECULE = "macromolecule";
SbgnPDConstants.UNIT_OF_INFORMATION = "unit of information";
SbgnPDConstants.STATE_VARIABLE = "state variable";
SbgnPDConstants.SOURCE_AND_SINK = "source and sink";
SbgnPDConstants.ASSOCIATION = "association";
SbgnPDConstants.DISSOCIATION = "dissociation";
SbgnPDConstants.OMITTED_PROCESS = "omitted process";
SbgnPDConstants.UNCERTAIN_PROCESS = "uncertain process";
SbgnPDConstants.SIMPLE_CHEMICAL = "simple chemical";
SbgnPDConstants.PROCESS = "process";
SbgnPDConstants.COMPLEX = "complex";
SbgnPDConstants.AND = "and";
SbgnPDConstants.OR = "or";
SbgnPDConstants.NOT = "not";
SbgnPDConstants.PHENOTYPE = "phenotype";
SbgnPDConstants.PERTURBING_AGENT = "perturbing agent";
SbgnPDConstants.TAG = "tag";
SbgnPDConstants.NUCLEIC_ACID_FEATURE = "nucleic acid feature";
SbgnPDConstants.UNSPECIFIED_ENTITY = "unspecified entity";
SbgnPDConstants.INPUT_PORT = "input_port";
SbgnPDConstants.OUTPUT_PORT = "output_port";

/**
 *  This compound type is only used to enclose a process node and its two associated port nodes 
 */
SbgnPDConstants.DUMMY_COMPOUND = "dummy compound";

// Below are the SBGN Arc specific types.
SbgnPDConstants.PRODUCTION = "production";
SbgnPDConstants.CONSUMPTION = "consumption";
SbgnPDConstants.INHIBITION = "inhibition";
SbgnPDConstants.CATALYSIS = "catalysis";
SbgnPDConstants.MODULATION = "modulation";
SbgnPDConstants.STIMULATION = "stimulation";
SbgnPDConstants.NECESSARY_STIMULATION = "necessary stimulation";

SbgnPDConstants.RIGID_EDGE_LENGTH = 10;
SbgnPDConstants.RIGID_EDGE = "rigid edge";

SbgnPDConstants.PORT_NODE_DEFAULT_WIDTH = 3;
SbgnPDConstants.PORT_NODE_DEFAULT_HEIGHT = 3;

SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER = 5;
SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER = 5;
SbgnPDConstants.COMPLEX_MEM_MARGIN = 20;
SbgnPDConstants.COMPLEX_MIN_WIDTH = SbgnPDConstants.COMPLEX_MEM_MARGIN * 2;	

SbgnPDConstants.PHASE1_MAX_ITERATION_COUNT = 200;
SbgnPDConstants.APPROXIMATION_DISTANCE = 10;
SbgnPDConstants.APPROXIMATION_PERIOD = 30;
SbgnPDConstants.PHASE2_INITIAL_COOLINGFACTOR = 0.3;

SbgnPDConstants.EFFECTOR_ANGLE_TOLERANCE = 45;
SbgnPDConstants.ANGLE_TOLERANCE = 100;
SbgnPDConstants.ROTATION_90_DEGREE = 60;
SbgnPDConstants.ROTATION_180_DEGREE = 0.5;
SbgnPDConstants.ROTATIONAL_FORCE_ITERATION_COUNT = 2;
SbgnPDConstants.ROTATIONAL_FORCE_CONVERGENCE = 1.0;

module.exports = SbgnPDConstants;
},{"./CoSEConstants":1}],36:[function(_dereq_,module,exports){
var CoSEEdge = _dereq_('./CoSEEdge');
var SbgnPDConstants = _dereq_('./SbgnPDConstants');

function SbgnPDEdge(source, target, vEdge) 
{
    CoSEEdge.call(this, source, target, vEdge);
    
    this.correspondingAngle = 0;
    this.isProperlyOriented = false;
}

function SbgnPDEdge(source, target, vEdge, type) 
{
    CoSEEdge.call(this, source, target, vEdge);
    
    this.type = type;                // String (from LGraphObject)
    this.correspondingAngle = 0;     // int
    this.isProperlyOriented = false; // boolean
}

CoSEEdge.prototype = Object.create(CoSEEdge.prototype);
for (var prop in CoSEEdge) {
  SbgnPDEdge[prop] = CoSEEdge[prop];
}

SbgnPDEdge.prototype.copy = function (/*SbgnPDEdge*/ edge) 
{
    // TODO: Do we really have access to these two functions?
    this.setSource(edge.getSource());
    this.setTarget(edge.getTarget());
    
    this.label = edge.label;
    this.type = edge.type;
    this.correspondingAngle = edge.correspondingAngle;
    this.isProperlyOriented = edge.isProperlyOriented;
    this.idealLength = edge.idealLength;
    this.isInterGraph = edge.isInterGraph;
    this.bendpoints = edge.bendpoints;
    this.isOverlapingSourceAndTarget = edge.isOverlapingSourceAndTarget;
    this.lca = edge.lca;
    this.length = edge.length;
    this.lengthX = edge.lengthX;
    this.lengthY = edge.lengthY;
    this.sourceInLca = edge.sourceInLca;
};

SbgnPDEdge.prototype.isEffector = function () 
{
    if(this.type.localeCompare(SbgnPDConstants.MODULATION) === 0 || 
       this.type.localeCompare(SbgnPDConstants.STIMULATION) === 0 || 
       this.type.localeCompare(SbgnPDConstants.CATALYSIS) === 0 || 
       this.type.localeCompare(SbgnPDConstants.INHIBITION) === 0 || 
       this.type.localeCompare(SbgnPDConstants.NECESSARY_STIMULATION) === 0)
   {
       return true;
   }
   else
   {
       return false;
   }
};

SbgnPDEdge.prototype.isRigidEdge = function () 
{
    if(this.type.localeCompare(SbgnPDConstants.RIGID_EDGE) === 0 )
    {
        return true;
    }
    else
    {
        return false;
    }
};

module.exports = SbgnPDEdge;

},{"./CoSEEdge":2,"./SbgnPDConstants":35}],37:[function(_dereq_,module,exports){
var Integer = _dereq_('./Integer');
var IGeometry = _dereq_('./IGeometry');
var PointD = _dereq_('./PointD');
var RectangleD = _dereq_('./RectangleD');

var HashMap = _dereq_('./HashMap');
var HashSet = _dereq_('./HashSet');

var CoSELayout = _dereq_('./CoSELayout');
var SbgnPDNode = _dereq_('./SbgnPDNode');
var SbgnPDEdge = _dereq_('./SbgnPDEdge');
var SbgnProcessNode = _dereq_('./SbgnProcessNode');
var SbgnPDConstants = _dereq_('./SbgnPDConstants');

var MemberPack = _dereq_('./MemberPack');
var RectProc = _dereq_('./RectProc');
var Compaction = _dereq_('./Compaction');

SbgnPDLayout.DefaultCompactionAlgorithmEnum = 
{
    TILING : 0, 
    POLYOMINO_PACKING : 1
};

SbgnPDLayout.OrientationEnum = 
{
    LEFT_TO_RIGHT : 0, 
    RIGHT_TO_LEFT : 1,
    TOP_TO_BOTTOM : 2, 
    BOTTOM_TO_TOP : 3
};

function SbgnPDLayout() 
{
    CoSELayout.call(this);

    this.rotationRandomizationMethod = 1;

    this.enhancedRatio = 0;
    this.totalEffCount = 0;
    this.compactionMethod = SbgnPDLayout.DefaultCompactionAlgorithmEnum.TILING;

    this.childGraphMap = new HashMap();          /*Map<SbgnPDNode, LGraph>*/
    this.complexOrder = [];                      /*List<SbgnPDNode>*/
    this.dummyComplexList = [];                  /*List<SbgnPDNode>*/
    this.emptiedDummyComplexMap = new HashMap(); /*Map<SbgnPDNode, LGraph>*/
    this.processNodeList = [];                   /*List<SbgnProcessNode>*/
    
    if (this.compactionMethod === SbgnPDLayout.DefaultCompactionAlgorithmEnum.TILING)
    {
        this.memberPackMap = new HashMap();      /*Map<SbgnPDNode, MemberPack>*/
    }
};

SbgnPDLayout.prototype = Object.create(CoSELayout.prototype);

for (var prop in CoSELayout) {
  SbgnPDLayout[prop] = CoSELayout[prop];
}

/**
 * @Override This method performs the actual layout on the l-level compound
 *           graph. An update() needs to be called for changes to be
 *           propagated to the v-level compound graph.
 */
SbgnPDLayout.prototype.runSpringEmbedder = function ()
{
    console.log("SBGN-PD Layout is running...");
    this.phaseNumber = 1;
    this.doPhase1();

    this.phaseNumber = 2;
    this.doPhase2();

    // used to calculate - to make sure
    this.recalcProperlyOrientedEdges(true);

    console.log("success ratio: " + this.successRatio);

    this.finalEnhancement();

    console.log("enhanced ratio: " + this.enhancedRatio);

    this.removeDummyCompounds();
};

/**
* At this phase, CoSE is applied for a number of iterations.
*/
SbgnPDLayout.prototype.doPhase1 = function ()
{
    this.maxIterations = SbgnPDConstants.PHASE1_MAX_ITERATION_COUNT;
    this.totalIterations = 0;

    do
    {
        this.totalIterations++;
        if ((this.totalIterations % SbgnPDConstants.CONVERGENCE_CHECK_PERIOD) === 0)
        {
            if (this.isConverged())
            {
                break;
            }

            this.coolingFactor = 
                this.initialCoolingFactor * 
                ((this.maxIterations - this.totalIterations) / this.maxIterations);
        }

        this.totalDisplacement = 0;

        this.graphManager.updateBounds();
        this.calcSpringForces();
        this.calcRepulsionForces();
        this.calcGravitationalForces();
        this.moveNodes();

        this.animate();
    }
    while (this.totalIterations < this.maxIterations);

    this.graphManager.updateBounds();
    this.phase1IterationCount = this.totalIterations;
};

/**
* At this phase, location of single nodes are approximated occasionally.
* Rotational forces are applied. Cooling factor starts from a small value
* to prevent huge changes.
*/
SbgnPDLayout.prototype.doPhase2 = function ()
{
    // dynamic max iteration
    this.maxIterations = 
            Math.log(this.getAllEdges().length + this.getAllNodes().length) * 400;

    // cooling fac is small
    this.initialCoolingFactor = SbgnPDConstants.PHASE2_INITIAL_COOLINGFACTOR;
    this.coolingFactor = this.initialCoolingFactor;

    this.totalIterations = 0;

    do
    {
        this.totalIterations++;

        if ((this.totalIterations % SbgnPDConstants.CONVERGENCE_CHECK_PERIOD) === 0)
        {
                this.successRatio = this.properlyOrientedEdgeCount / this.totalEdgeCountToBeOriented;

                if (this.isConverged() && 
                    successRatio >= SbgnPDConstants.ROTATIONAL_FORCE_CONVERGENCE)
                {
                    break;
                }

                this.coolingFactor = 
                        this.initialCoolingFactor * 
                        ((this.maxIterations - this.totalIterations) / this.maxIterations);
        }

        this.totalDisplacement = 0;

        this.graphManager.updateBounds();

        this.calcSpringForces();
        this.calcRepulsionForces();
        this.calcGravitationalForces();
        this.moveNodes();
        this.animate();
    }
    while (this.totalIterations < this.maxIterations
                    && this.totalIterations < 10000);

    this.phase2IterationCount = this.totalIterations;
    this.graphManager.updateBounds();
};

SbgnPDLayout.prototype.moveNodes = function ()
{
    this.properlyOrientedEdgeCount = 0;
    this.totalEdgeCountToBeOriented = 0;

    // only change single node positions on early stages
    if (this.hasApproximationPeriodReached() && this.coolingFactor > 0.02)
    {
        var numOfProcessNodes = this.processNodeList.length;
        for (var index; index<numOfProcessNodes; index++)
        {
            this.processNodeList[index].applyApproximations();
        }
    }
    
    var numOfProcessNodes = this.processNodeList.length;
    for (var index; index<numOfProcessNodes; index++)
    {
        var sbgnProcessNode = this.processNodeList[index];
        
        // calculate rotational forces for phase 2 only
        if (this.phaseNumber === 2)
        {
            sbgnProcessNode.calcRotationalForces();

            this.properlyOrientedEdgeCount += sbgnProcessNode.properEdgeCount;
            this.totalEdgeCountToBeOriented += 
                    (sbgnProcessNode.consumptionEdges.size() + 
                     sbgnProcessNode.productEdges.size() + 
                     sbgnProcessNode.effectorEdges.size());
            this.successRatio = 
                    this.properlyOrientedEdgeCount / this.totalEdgeCountToBeOriented;
        }
        
        sbgnProcessNode.transferForces();

        sbgnProcessNode.resetForces();
        sbgnProcessNode.inputPort.resetForces();
        sbgnProcessNode.outputPort.resetForces();
    }

    // each time, rotate one process that wants to rotate
    if (((this.totalIterations % SbgnPDConstants.ROTATIONAL_FORCE_ITERATION_COUNT) === 0) && 
        this.phaseNumber === 2)
    {
        rotateAProcess();
    }
            
    // TODO: Original: super.moveNodes();
    CoSELayout.moveNodes();
};

SbgnPDLayout.prototype.hasApproximationPeriodReached = function ()
{
    if((this.totalIterations % 100) === (SbgnPDConstants.APPROXIMATION_PERIOD))
    {
        return true;
    }
    else
    {
        return false;
    }
};

SbgnPDLayout.prototype.rotateAProcess = function ()
{
    var processNodesToBeRotated = [] /*List<SbgnProcessNode>*/;
    
    var numOfProcessNodes = this.processNodeList.length;
    for (var index; index<numOfProcessNodes; index++)
    {
        var sbgnProcessNode = this.processNodeList[index];
        
        if (sbgnProcessNode.isRotationNecessary())
        {
            processNodesToBeRotated.push(sbgnProcessNode);
        }       
    }

    // random selection
    if (processNodesToBeRotated.length > 0)
    {
        var randomIndex = 0;

        if (this.rotationRandomizationMethod === 0)
        {
            randomIndex = this.rouletteWheelSelection(processNodesToBeRotated);

            if (randomIndex === -1)
            {
                console.log("ERROR: no nodes have been selected for rotation");
            }
        }
        else
        {
            randomIndex = (Math.random() * processNodesToBeRotated.length);
        }

        var sbgnProcessNode = processNodesToBeRotated[randomIndex];
        sbgnProcessNode.applyRotation();
    }

    // // reset net rotational forces on all processes for next round
    // not used because even if the amount is small, summing up the net
    // force from prev iterations yield better results
    // for (Object o : this.getAllNodes())
    // {
    // if (o instanceof SbgnProcessNode)
    // ((SbgnProcessNode) o).netRotationalForce = 0;
    // }
};

/**
* This method iterates over the process nodes and checks if there exists
* another orientation which maximizes the total number of properly edges.
* If there is, the orientation is changed.
*/
SbgnPDLayout.prototype.finalEnhancement = function ()
{
    var orientationList = [] /*List<Orientation>*/;
    var bestStepResult = 0.0;
    var bestOrientation = null /*Orientation*/;
    var stepAppropriateEdgeCnt = 0.0;
    var totalProperEdges = 0.0;
    var angle = 0.0;

    orientationList.push(SbgnPDLayout.OrientationEnum.LEFT_TO_RIGHT);
    orientationList.push(SbgnPDLayout.OrientationEnum.RIGHT_TO_LEFT);
    orientationList.push(SbgnPDLayout.OrientationEnum.TOP_TO_BOTTOM);
    orientationList.push(SbgnPDLayout.OrientationEnum.BOTTOM_TO_TOP);
    
    var numOfProcessNodes = this.processNodeList.length;
    for (var i; i<numOfProcessNodes; i++)
    {
        var sbgnProcessNode = this.processNodeList[i];
    
        bestStepResult = sbgnProcessNode.properEdgeCount;
        bestOrientation = null;
        var rememberPropList = []/*List<Boolean>*/;
        var bestPropList = []/*List<Boolean>*/;
        
        var numOfOrientations = orientationList.length;
        for (var j; j<numOfOrientations; j++)
        {
            var orient = orientationList[j];
            
            stepAppropriateEdgeCnt = 0;

            var inputPortTarget = sbgnProcessNode.findPortTargetPoint(true, orient);
            var outputPortTarget = sbgnProcessNode.findPortTargetPoint(false, orient);

            rememberPropList = [];
            
            var numOfConsumptionEdges = sbgnProcessNode.consumptionEdges.length;
            for (var k; k<numOfConsumptionEdges; k++)
            {
                var edge = sbgnProcessNode.consumptionEdges[k];
                var node = edge.getSource();
                angle = IGeometry.calculateAngle(inputPortTarget,
                                sbgnProcessNode.inputPort.getCenter(), 
                                node.getCenter());
                if (angle <= SbgnPDConstants.ANGLE_TOLERANCE)
                {
                    stepAppropriateEdgeCnt++;
                    rememberPropList.push(true);
                }
                else
                {
                    rememberPropList.push(false);
                }
            }
            
            var numOfProductEdges = sbgnProcessNode.productEdges.length;
            for (var k; k<numOfProductEdges; k++)
            {
                var edge = sbgnProcessNode.productEdges[k];
                var node = edge.getTarget();
                angle = IGeometry.calculateAngle(outputPortTarget,
                                sbgnProcessNode.outputPort.getCenter(), 
                                node.getCenter());

                if (angle <= SbgnPDConstants.ANGLE_TOLERANCE)
                {
                    stepAppropriateEdgeCnt++;
                    rememberPropList.push(true);
                }
                else
                {
                    rememberPropList.push(false);
                }         
            }
            
            var numOfEffectorEdges = sbgnProcessNode.effectorEdges.length;
            for (var k; k<numOfEffectorEdges; k++)
            {
                var edge = sbgnProcessNode.effectorEdges[k];
                var node = edge.getSource();
                angle = calcEffectorAngle(orient, sbgnProcessNode.getCenter(), node);

                if (angle <= SbgnPDConstants.EFFECTOR_ANGLE_TOLERANCE)
                {
                    stepAppropriateEdgeCnt++;
                    rememberPropList.push(true);

                }
                else
                {
                    rememberPropList.push(false);
                }
            }

            if (stepAppropriateEdgeCnt > bestStepResult)
            {
                bestStepResult = stepAppropriateEdgeCnt;
                bestOrientation = orient;
                bestPropList = rememberPropList;
            }
        }
        totalProperEdges += bestStepResult;

        // it means a better position has been found
        if (bestStepResult > sbgnProcessNode.properEdgeCount)
        {
            sbgnProcessNode.setOrientation(bestOrientation);
            sbgnProcessNode.properEdgeCount = bestStepResult;

            // mark edges with best known configuration values
            for (var m = 0; m < sbgnProcessNode.consumptionEdges.length; m++)
            {
                sbgnProcessNode.consumptionEdges[i].isProperlyOriented = bestPropList[i];
            }
            for (var n = 0; n < sbgnProcessNode.productEdges.length; n++)
            {
                sbgnProcessNode.productEdges[i].isProperlyOriented = bestPropList[i + sbgnProcessNode.consumptionEdges.length];
            }
            for (var o = 0; o < sbgnProcessNode.effectorEdges.size(); o++)
            {
                    sbgnProcessNode.effectorEdges[i].isProperlyOriented = 
                            bestPropList[i + 
                                (sbgnProcessNode.consumptionEdges.length + 
                                    sbgnProcessNode.productEdges.length)];
            }
        }
    }

    this.properlyOrientedEdgeCount = totalProperEdges;
    this.enhancedRatio = totalProperEdges / totalEdgeCountToBeOriented;

    var numOfProcessNodes = this.processNodeList.length;
    for (var i; i<numOfProcessNodes; i++)
    {
        totalEffCount += this.processNodeList[i].effectorEdges.length;
    }
};

/**
* If a process node has higher netRotationalForce, it has more chance to be
* rotated
*/
SbgnPDLayout.prototype.rouletteWheelSelection = function (
               /*ArrayList<SbgnProcessNode>*/ processNodesToBeRotated)
{
    var randomNumber = Math.random();
    var fitnessValues = [] /*processNodesToBeRotated.size()]*/;
    var totalSum = 0;
    var sumOfProbabilities = 0;
    var i = 0;

    var numOfProcessNodesToBeRotated = processNodesToBeRotated.length;
    for (var j; j<numOfProcessNodesToBeRotated; j++)
    {
        totalSum += Math.abs(processNodesToBeRotated[j].netRotationalForce);
    }

    // normalize all between 0..1
    var numOfProcessNodesToBeRotated = processNodesToBeRotated.length;
    for (var j; j<numOfProcessNodesToBeRotated; j++)
    {
        fitnessValues[i] = 
                sumOfProbabilities + 
                (Math.abs(processNodesToBeRotated[j].netRotationalForce) / 
                    totalSum);

        sumOfProbabilities = fitnessValues[i];
        i++;
    }
        
    if (randomNumber < fitnessValues[0])
    {
        return 0;
    }
    else
    {
        for (var j = 0; j < fitnessValues.length - 1; j++)
        {
            if ((randomNumber >= fitnessValues[j]) && 
                (randomNumber < fitnessValues[j+1]))
            {
                return j + 1;
            }
        }
    }

    return -1;
};

SbgnPDLayout.prototype.calcEffectorAngle = function (
        /*Orientation*/ orient, 
        /*PointD*/      centerPt,
        /*CoSENode*/    eff)
{
    var idealEdgeLength = this.idealEdgeLength;
    var targetPnt = new PointD();
    var centerPnt = centerPt;

    // find target point
    if (orient === SbgnPDLayout.OrientationEnum.LEFT_TO_RIGHT || 
        orient === SbgnPDLayout.OrientationEnum.RIGHT_TO_LEFT)
    {
        targetPnt.x = centerPnt.x;

        if (eff.getCenterY() > centerPnt.y)
        {
            targetPnt.y = centerPnt.y + idealEdgeLength;
        }
        else
        {
            targetPnt.y = centerPnt.y - idealEdgeLength;
        }
    }
    else if (orient === SbgnPDLayout.OrientationEnum.BOTTOM_TO_TOP || 
             orient === SbgnPDLayout.OrientationEnum.TOP_TO_BOTTOM)
    {
        targetPnt.y = centerPnt.y;

        if (eff.getCenterX() > centerPnt.x)
        {
            targetPnt.x = centerPnt.x + idealEdgeLength;
        }
        else
        {
            targetPnt.x = centerPnt.x - idealEdgeLength;
        }
    }

    var angle = IGeometry.calculateAngle(targetPnt, centerPnt, eff.getCenter());

    return angle;
};

/**
* Recursively calculate if the node or its child nodes have any edges to
* other nodes. Return the total number of edges.
*/
SbgnPDLayout.prototype.calcGraphDegree = function (/*SbgnPDNode*/ parentNode)
{
    var degree = 0;
    if (parentNode.getChild() == null)
    {
        degree = parentNode.getEdges().length;
        return degree;
    }

    var numOfChildren = parentNode.getChild().getNodes().length;
    for (var i=0; i<numOfChildren; i++)
    {
        degree = degree + parentNode.getEdges().length
                        + this.calcGraphDegree(parentNode.getChild().getNodes()[i]);
    }
    
    return degree;
};

SbgnPDLayout.prototype.recalcProperlyOrientedEdges = function (/*boolean*/ isLastIteration)
{
    this.properlyOrientedEdgeCount = 0.0;
    this.totalEdgeCountToBeOriented = 0;
    
    var numOfProcessNodes = this.processNodeList.length;
    for(var i=0; i<numOfProcessNodes; i++)
    {
        var sbgnProcessNode = this.processNodeList[i];
        
        sbgnProcessNode.calcProperlyOrientedEdges();
        this.properlyOrientedEdgeCount += sbgnProcessNode.properEdgeCount;
        this.totalEdgeCountToBeOriented += 
                (sbgnProcessNode.consumptionEdges.length + 
                 sbgnProcessNode.productEdges.length + 
                 sbgnProcessNode.effectorEdges.length);
        this.successRatio = 
                this.properlyOrientedEdgeCount / this.totalEdgeCountToBeOriented;
    }
};

/**
* This method finds all the zero degree nodes in the graph which are not
* owned by a complex node. Zero degree nodes at each level are grouped
* together and placed inside a dummy complex to reduce bounds of root
* graph.
*/
SbgnPDLayout.prototype.groupZeroDegreeMembers = function ()
{
    var childComplexMap = new HashMap();/*SbgnPDNode, LGraph*/
    
    var numOfGraphs = this.getGraphManager().getGraphs().length;
    for (var i=0; i<numOfGraphs; i++)
    {
        var ownerGraph = this.getGraphManager().getGraphs()[i];
        var zeroDegreeNodes = []; /*ArrayList<SbgnPDNode>*/
        
        // do not process complex nodes (their members are already owned)
        if ((ownerGraph.getParent().type !== null) && 
            (ownerGraph.getParent().isComplex()))
        {
            continue;
        }
        
        var numOfNodes = ownerGraph.getNodes().length;
        for (var j=0; j<numOfNodes; j++)
        {
            var node = ownerGraph.getNodes()[j];

            if (this.calcGraphDegree(node) === 0)
            {
                zeroDegreeNodes.push(node);
            }
        }

        if (zeroDegreeNodes.length > 1)
        {
            // create a new dummy complex
            var complex = this.newNode(null);
            complex.type = SbgnPDConstants.COMPLEX;
            complex.label = "DummyComplex_" + ownerGraph.getParent().label;

            ownerGraph.add(complex);

            var childGraph = newGraph(null);
            
            var numOfZeroDegreeNode = zeroDegreeNodes.length;
            for (var j=0; j<numOfZeroDegreeNode; j++)
            {
                var zeroNode = zeroDegreeNodes[j];
                ownerGraph.remove(zeroNode);
                childGraph.add(zeroNode);
            }
            
            this.dummyComplexList.push(complex);
            childComplexMap.put(complex, childGraph);
        }
    }
    
    var numOfComplexNodes = this.dummyComplexList.length;
    for (var i=0; i<numOfComplexNodes; i++)
    {
        this.graphManager.add(childComplexMap.get(complex), this.dummyComplexList[i]);
    }
    
    this.getGraphManager().updateBounds();

    this.graphManager.resetAllNodes();
    this.graphManager.resetAllNodesToApplyGravitation();
    this.graphManager.resetAllEdges();
    this.calculateNodesToApplyGravitationTo();
};

/**
* This method creates two port nodes and a compound for each process nodes
* and adds them to graph.
*/
SbgnPDLayout.prototype.createPortNodes = function ()
{
    var numOfNodes = this.getAllNodes().length;
    for (var i=0; i<numOfNodes; i++)
    {
        var originalProcessNode = this.getAllNodes()[i];

        if (originalProcessNode.type === SbgnPDConstants.PROCESS)
        {
            var ownerGraph = originalProcessNode.getOwner();

            // create new nodes and graphs
            var processNode = newProcessNode(null);
            var inputPort   = newPortNode(null, SbgnPDConstants.INPUT_PORT);
            var outputPort  = newPortNode(null, SbgnPDConstants.OUTPUT_PORT);

            // create a dummy compound
            var compoundNode = newNode(null);
            compoundNode.type = SbgnPDConstants.DUMMY_COMPOUND;

            // add labels
            compoundNode.label = "DummyCompound_" + originalProcessNode.label;
            inputPort.label = "InputPort_" + originalProcessNode.label;
            outputPort.label = "OutputPort_" + originalProcessNode.label;

            // create child graph (= 2port+process) to be set as child to
            // dummy compound
            var childGraph = newGraph(null);
            ownerGraph.add(processNode);

            // convert the process node to SbgnProcessNode
            processNode.copyFromSBGNPDNode(originalProcessNode,
                            this.getGraphManager());

            processNode.connectNodes(compoundNode, inputPort, outputPort);

            // create rigid edges, change edge connections
            processNode.reconnectEdges(idealEdgeLength);

            var rigidToProduction = newRigidEdge(null);
            rigidToProduction.label = ""
                            + (this.graphManager.getAllEdges().length + 1);

            var rigidToConsumption = newRigidEdge(null);
            rigidToConsumption.label = ""
                            + (this.graphManager.getAllEdges().length + 2);

            ownerGraph.remove(processNode);

            // organize child graph
            childGraph.add(processNode);
            childGraph.add(inputPort);
            childGraph.add(outputPort);
            childGraph.add(rigidToProduction, inputPort, processNode);
            childGraph.add(rigidToConsumption, outputPort, processNode);

            // organize the compound node
            compoundNode.setOwner(ownerGraph);
            compoundNode.setCenter(processNode.getCenterX(),
                            processNode.getCenterY());
            ownerGraph.add(compoundNode);
            this.graphManager.add(childGraph, compoundNode);

            // remove the original process node
            ownerGraph.remove(originalProcessNode);

            this.processNodeList.push(processNode);
            this.graphManager.updateBounds();
        }
    }

    // reset the topology
    this.graphManager.resetAllNodes();
    this.graphManager.resetAllNodesToApplyGravitation();
    this.graphManager.resetAllEdges();

    this.calculateNodesToApplyGravitationTo();
};

/**
* This method checks whether there exists any process nodes in the graph.
* If there exist any process nodes it is assumed that the given graph
* respects our structure.
* 
* Most likely: this method does not work properly. Never had any input to
* test. Not complete.
*/
SbgnPDLayout.prototype.arePortNodesCreated = function ()
{
    var flag = false;

    // if there are any process nodes, check for port nodes
    var numOfNodes = this.getAllNodes().length;
    for (var i=0; i<numOfNodes; i++)
    {
        var sbgnPDNode = this.getAllNodes()[i];
        
        if (sbgnPDNode.type === SbgnPDConstants.PROCESS)
        {
            flag = true;
            break;
        }
    }
    
    // if there are no process nodes, no need to check for port nodes
    if (!flag)
    {
        return true;
    }
    else
    {
        // check for the port nodes. if any found, return true.
        var numOfNodes = this.getAllNodes().length;
        for (var i=0; i<numOfNodes; i++)
        {
            var sbgnPDNode = this.getAllNodes()[i];
            
            if (sbgnPDNode.type === SbgnPDConstants.INPUT_PORT || 
                sbgnPDNode.type === SbgnPDConstants.OUTPUT_PORT)
            {
                return true;
            }
        }
    }
    
    return false;
};

/**
* This method is used to remove the dummy compounds (previously created for
* each process node) from the graph.
*/
SbgnPDLayout.prototype.removeDummyCompounds = function ()
{
    var numOfProcessNodes = this.processNodeList.length;
    for (var i=0; i<numOfProcessNodes; i++)
    {
        var processNode = this.processNodeList[i];
        var dummyNode = processNode.parentCompound;
        var childGraph = dummyNode.getChild();
        var owner = dummyNode.getOwner();

        // add children to original parent
        var numOfChildNodes = childGraph.getNodes().length;
        for (var j=0; j<numOfChildNodes; j++)
        {
            owner.add(childGraph.getNodes()[j]);
        }

        var numOfChildEdges = childGraph.getEdges().length;
        for (var j=0; j<numOfChildEdges; j++)
        {
            var edge = childGraph.getEdges()[j]
            owner.add(edge, edge.getSource(), edge.getTarget());
        }
        
        // add effectors / remaining edges back to the process
        for (var j = 0; j < dummyNode.getEdges().length; j++)
        {
            var edge = dummyNode.getEdges()[j];

            dummyNode.getEdges().splice(j, 1);
            edge.setTarget(processNode);
            processNode.getEdges().push(edge);
            j--;
        }

        // remove the graph
        this.getGraphManager().getGraphs().splice(this.getGraphManager().getGraphs().indexOf(childGraph), 1);
        dummyNode.setChild(null);
        owner.remove(dummyNode);
    }

    this.getGraphManager().resetAllNodes();
    this.getGraphManager().resetAllNodesToApplyGravitation();
    this.getGraphManager().resetAllEdges();
    this.calculateNodesToApplyGravitationTo();
};


// ********************* SECTION : TILING METHODS *********************

SbgnPDLayout.prototype.clearComplex = function (/*SbgnPDNode*/ comp)
{
    var pack = null; /* MemberPack */
    var childGr = comp.getChild(); /* LGraph */
    this.childGraphMap.put(comp, childGr);

    if (childGr == null)
    {
        return;
    }
    
    if (this.compactionMethod == SbgnPDLayout.DefaultCompactionAlgorithmEnum.POLYOMINO_PACKING)
    {
        this.applyPolyomino(comp);
    }
    else if (this.compactionMethod == SbgnPDLayout.DefaultCompactionAlgorithmEnum.TILING)
    {
        pack = new MemberPack(childGr);
        this.memberPackMap.put(comp, pack);
    }

    if (this.dummyComplexList.includes(comp))
    {
        for (var i=0; i<comp.getChild().getNodes().length; i++)
        {
            clearDummyComplexGraphs(comp.getChild().getNodes()[i]);
        }
    }

    var remIndex = this.getGraphManager().getGraphs().indexOf(childGr);
    this.getGraphManager().getGraphs().splice(remIndex, 1);
    comp.setChild(null);

    if (this.compactionMethod == SbgnPDLayout.DefaultCompactionAlgorithmEnum.TILING)
    {
        comp.setWidth(pack.getWidth());
        comp.setHeight(pack.getHeight());
    }

    // Redirect the edges of complex members to the complex.
    if (childGr != null)
    {
        for (var i=0; i<childGr.getNodes().length; i++)
        {
            var chNd = childGr.getNodes()[i];

            for (var j=0; j<chNd.getEdges().length; j++)
            {
                var edge = chNd.getEdges()[j];
                if (edge.getSource() == chNd)
                {
                    chNd.getEdges().splice(chNd.getEdges().indexOf(edge), 1);
                    edge.setSource(comp);
                    comp.getEdges().push(edge);
                }
                else if (edge.getTarget() == chNd)
                {
                    chNd.getEdges().splice(chNd.getEdges().indexOf(edge), 1);
                    edge.setTarget(comp);
                    comp.getEdges().push(edge);
                }
            }
        }
    }
};

/**
* This method searched unmarked complex nodes recursively, because they may
* contain complex children. After the order is found, child graphs of each
* complex node are cleared.
*/
SbgnPDLayout.prototype.applyDFSOnComplexes = function ()
{
       // LGraph>();
       var numOfNodes = this.getAllNodes().length;
       for (var i = 0; i < numOfNodes; i++)
       {
            var node = this.getAllNodes()[i];
           
            // TODO: Instance of!
            if (node.isComplex())
            {
                continue;
            }
            // complex is found, recurse on it until no visited complex remains.
            if (!node.visited)
            {
                this.DFSVisitComplex(node);
            }
       }

       // clear each complex
       var numOfComplexOrder = this.complexOrder.length;
       for (var i = 0; i < numOfComplexOrder; i++)
       {
           clearComplex(this.complexOrder[i]);
       }

       this.getGraphManager().updateBounds();

       this.getGraphManager().resetAllNodes();
       this.getGraphManager().resetAllNodesToApplyGravitation();
       this.getGraphManager().resetAllEdges();
};

/**
* This method recurses on the complex objects. If a node does not contain
* any complex nodes or all the nodes in the child graph is already marked,
* it is reported. (Depth first)
* 
*/
SbgnPDLayout.prototype.DFSVisitComplex = function (/*SbgnPDNode*/ node)
{
    if (node.getChild() != null)
    {
         var numOfChildren =  node.getChild().getNodes().length;
         for (var i = 0; i < numOfChildren; i++)
         {
             this.DFSVisitComplex(node.getChild().getNodes()[i]);
         }
    }

    if (node.isComplex() && !node.containsUnmarkedComplex())
    {
         this.complexOrder.push(node);
         node.visited = true;
         return;
    }
};

/**
* This method tiles the given list of nodes by using polyomino packing
* algorithm.
*/
SbgnPDLayout.prototype.applyPolyomino = function (/*SbgnPDNode*/ parent)
{
    var rect;
    var childGr = parent.getChild();

    if (childGr == null)
    {
        console.log("Child graph is empty (Polyomino)");
    }
    else
    {
        // packing takes the input as an array. put the members in an array.
        var mpArray = [];
        for (var i = 0; i < childGr.getNodes().length; i++)
        {
            mpArray[i] = childGr.getNodes()[i];
        }

        // pack rectangles
        RectProc.packRectanglesMino(
                        SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER,
                        mpArray.length, 
                        mpArray);

        // apply compaction
        var c = new Compaction(childGr.getNodes());
        c.perform();

        // get the resulting rectangle and set parent's (complex) width &
        // height
        rect = this.calculateBounds(true, childGr.getNodes());

        parent.setWidth(rect.getWidth());
        parent.setHeight(rect.getHeight());
    }
};

/**
* Reassigns the complex content. The outermost complex is placed first.
*/
SbgnPDLayout.prototype.repopulateComplexes = function ()
{
    var emptiedDummyComplexMapSize = this.emptiedDummyComplexMap.keySet().length;
    for (var i = 0; i < emptiedDummyComplexMapSize; i++)
    {
        var comp = this.emptiedDummyComplexMap.keySet()[i];
        var chGr = this.emptiedDummyComplexMap.get(comp);
        comp.setChild(chGr);
        this.getGraphManager().getGraphs().push(chGr);
    }

    for (var i = this.complexOrder.length - 1; i >= 0; i--)
    {
        var comp = this.complexOrder[i];
        var chGr = this.childGraphMap.get(comp);

        // repopulate the complex
        comp.setChild(chGr);

        // if the child graph is not null, adjust the positions of members
        if (chGr != null)
        {
            // adjust the positions of the members
            if (this.compactionMethod === SbgnPDLayout.DefaultCompactionAlgorithmEnum.POLYOMINO_PACKING)
            {
                this.adjustLocation(comp, chGr);
                this.getGraphManager().getGraphs().push(chGr);
            }
            else if (compactionMethod === SbgnPDLayout.DefaultCompactionAlgorithmEnum.TILING)
            {
                this.getGraphManager().getGraphs().push(chGr);

                var pack = this.memberPackMap.get(comp);
                pack.adjustLocations(comp.getLeft(), comp.getTop());
            }
        }
    }
    
    var emptiedDummyComplexMapSize = this.emptiedDummyComplexMap.keySet().length;
    for (var i = 0; i < emptiedDummyComplexMapSize; i++)
    {
        var comp = this.emptiedDummyComplexMap.keySet()[i];
        var chGr = this.emptiedDummyComplexMap.get(comp);

        this.adjustLocation(comp, chGr);
    }

    this.removeDummyComplexes();

    // reset
    this.getGraphManager().resetAllNodes();
    this.getGraphManager().resetAllNodesToApplyGravitation();
    this.getGraphManager().resetAllEdges();
    this.calculateNodesToApplyGravitationTo();
};

/**
* Adjust locations of children of given complex wrt. the location of the
* complex
*/
SbgnPDLayout.prototype.adjustLocation = function (comp, chGr)
{
    var rect = calculateBounds(false, chGr.getNodes());

    var differenceX = (rect.x - comp.getLeft());
    var differenceY = (rect.y - comp.getTop());

    // if the parent graph is a compound, add compound margins
    if (comp.type !== SbgnPDConstants.COMPLEX)
    {
        differenceX -= SbgnPDConstants.COMPOUND_NODE_MARGIN;
        differenceY -= SbgnPDConstants.COMPOUND_NODE_MARGIN;
    }

    for (var j = 0; j < chGr.getNodes().length; j++)
    {
        var s = chGr.getNodes[j];

        s.setLocation(s.getLeft() - differenceX
                        + SbgnPDConstants.COMPLEX_MEM_HORIZONTAL_BUFFER, s.getTop()
                        - differenceY + SbgnPDConstants.COMPLEX_MEM_VERTICAL_BUFFER);

        if (s.getChild() !== null)
        {
            this.adjustLocation(s, s.getChild());
        }
    }
};

/**
* Recursively removes all dummy complex nodes (previously created to tile
* group degree-zero nodes) from the graph.
*/
SbgnPDLayout.prototype.clearDummyComplexGraphs = function (comp)
{
    if (comp.getChild() == null || comp.isDummyCompound)
    {
        return;
    }
    
    var numOfChildren = comp.getChild().getNodes().length;
    for (var i = 0; i < numOfChildren; i++)
    {
        var childNode = comp.getChild().getNodes()[i];
        if (childNode.getChild() != null && childNode.getEdges().length == 0)
        {
            this.clearDummyComplexGraphs(childNode);
        }
    }
    if (this.graphManager.getGraphs().includes(comp.getChild()))
    {
        if (this.calcGraphDegree(comp) === 0)
        {
            this.emptiedDummyComplexMap.put(comp, comp.getChild());

            this.getGraphManager().getGraphs().splice(
                    this.getGraphManager().getGraphs().indexOf(comp.getChild()), 1);
            comp.setChild(null);
        }
    }
};

/**
* Dummy complexes (placed in the "dummyComplexList") are removed from the
* graph.
*/
SbgnPDLayout.prototype.removeDummyComplexes = function ()
{
    var dummyComplexListSize = this.dummyComplexList.length;
    // remove dummy complexes and connect children to original parent
    for (var i=0; i<dummyComplexListSize; i++)
    {
        var dummyComplex = this.dummyComplexList[i];
        var childGraph = dummyComplex.getChild();
        var owner = dummyComplex.getOwner();

        this.getGraphManager().getGraphs().splice(
                this.getGraphManager().getGraphs().indexOf(childGraph), 1);
        dummyComplex.setChild(null);

        owner.remove(dummyComplex);

        var numOfChildren = childGraph.getNodes().length;
        for (var j=0; j<numOfChildren; j++)
        {
            owner.add(childGraph.getNodes()[j]);
        }
    }
};

/**
* This method returns the bounding rectangle of the given set of nodes with
* or without the margins
*/
SbgnPDLayout.prototype.calculateBounds = function (isMarginIncluded, nodes)
{
    var boundLeft = Integer.MAX_VALUE;
    var boundRight = Integer.MIN_VALUE;
    var boundTop = Integer.MAX_VALUE;
    var boundBottom = Integer.MIN_VALUE;
    var nodeLeft;
    var nodeRight;
    var nodeTop;
    var nodeBottom;
    
    var numOfChildren = nodes.length;
    for (var i=0; i<numOfChildren; i++)
    {
        var lNode = nodes[i];
        nodeLeft = lNode.getLeft();
        nodeRight = lNode.getRight();
        nodeTop = lNode.getTop();
        nodeBottom = lNode.getBottom();

        if (boundLeft > nodeLeft)
            boundLeft = nodeLeft;

        if (boundRight < nodeRight)
            boundRight = nodeRight;

        if (boundTop > nodeTop)
            boundTop = nodeTop;

        if (boundBottom < nodeBottom)
            boundBottom = nodeBottom;
    }

    if (isMarginIncluded)
    {
        return new RectangleD(boundLeft - SbgnPDConstants.COMPLEX_MEM_MARGIN, 
                              boundTop - SbgnPDConstants.COMPLEX_MEM_MARGIN, 
                              boundRight - boundLeft + 2 * SbgnPDConstants.COMPLEX_MEM_MARGIN,
                              boundBottom - boundTop + 2 * SbgnPDConstants.COMPLEX_MEM_MARGIN);
    }
    else
    {
        return new RectangleD(boundLeft, 
                              boundTop, 
                              boundRight - boundLeft, 
                              boundBottom - boundTop);
    }
};

/**
* calculates usedArea/totalArea inside the complexes and prints them out.
*/
SbgnPDLayout.prototype.calculateFullnessOfComplexes = function ()
{
    var largestComplex = null;
    var totalArea = 0.0;
    var usedArea = 0.0;
    var maxArea = Number.MIN_VALUE;

    // find the largest complex -> area
    for (var i = 0; i < this.getAllNodes().length; i++)
    {
        var s = this.getAllNodes()[i];
        if ((s.type === SbgnPDConstants.COMPLEX) && 
            ((s.getWidth() * s.getHeight()) > maxArea))
        {
            maxArea = s.getWidth() * s.getHeight();
            largestComplex = s;
        }
    }

    usedArea = this.calculateUsedArea(largestComplex);
    totalArea = largestComplex.getWidth() * largestComplex.getHeight();

    if (this.compactionMethod === SbgnPDLayout.DefaultCompactionAlgorithmEnum.TILING)
            console.log("Tiling results");
    else if (this.compactionMethod === SbgnPDLayout.DefaultCompactionAlgorithmEnum.POLYOMINO_PACKING)
            console.log("Polyomino Packing results");

    console.log(" = " + usedArea / totalArea);
};

/**
* This method calculates the used area of a given complex node's children
*/
SbgnPDLayout.prototype.calculateUsedArea = function (parent)
{
    var totalArea = 0;
    if (parent.getChild() == null)
    {
        return 0.0;
    }

    for (var i = 0; i < parent.getChild().getNodes().length; i++)
    {
        var node = parent.getChild().getNodes()[i];

        if (node.type !== SbgnPDConstants.COMPLEX)
        {
            totalArea += node.getWidth() * node.getHeight();
        }
        else
        {
            totalArea += this.calculateUsedArea(node);
        }
    }
    
    return totalArea;
};

// ********************* SECTION : OVERRIDEN METHODS *********************

/**
 * This method creates a new node associated with the input view node.
 */
SbgnPDLayout.prototype.newNode = function (vNode)
{
    return new SbgnPDNode(this.graphManager, null, null, vNode, null);
};

/**
 * This method creates a new edge associated with the input view edge.
 */
SbgnPDLayout.prototype.newEdge = function (vEdge)
{
    return new SbgnPDEdge(null, null, vEdge);
};

/**
 * This method performs layout on constructed l-level graph. It returns true
 * on success, false otherwise.
 */
SbgnPDLayout.prototype.layout = function ()
{
    var b = false;

    this.groupZeroDegreeMembers();
    this.applyDFSOnComplexes();
    b = CoSELayout.prototype.layout.call(this, arguments);
    this.repopulateComplexes();

    this.getAllNodes();
    return b;
};

/**
* This method uses classic layout method (without multi-scaling)
* Modification: create port nodes after random positioning
*/
//@Override
SbgnPDLayout.prototype.classicLayout = function ()
{
    this.calculateNodesToApplyGravitationTo();

    this.graphManager.calcLowestCommonAncestors();
    this.graphManager.calcInclusionTreeDepths();

    this.graphManager.getRoot().calcEstimatedSize();
    this.calcIdealEdgeLengths();

    if (!this.incremental)
    {
        var forest = this.getFlatForest();

        if (forest.length > 0)
        // The graph associated with this layout is flat and a forest
        {
            this.positionNodesRadially(forest);
        }
        else
        // The graph associated with this layout is not flat or a forest
        {
            this.positionNodesRandomly();
        }
    }

    if (!this.arePortNodesCreated())
    {
        this.createPortNodes();
        this.graphManager.resetAllNodes();
        this.graphManager.resetAllNodesToApplyGravitation();
        this.graphManager.resetAllEdges();
        this.calculateNodesToApplyGravitationTo();
    }
    
    this.initSpringEmbedder();
    this.runSpringEmbedder();

    return true;
};


/**
 * This method calculates the spring forces for the ends of each node.
 * Modification: do not calculate spring force for rigid edges
 */
//@Override
SbgnPDLayout.prototype.calcSpringForces = function ()
{
    var lEdges = this.getAllEdges();
    var edge;

    for (var i = 0; i < lEdges.length; i++)
    {
        edge = lEdges[i];

        if (edge.type !== SbgnPDConstants.RIGID_EDGE)
        {    
            this.calcSpringForce(edge, edge.idealLength);
        }
    }
};

/**	 
 * This method calculates the repulsion forces for each pair of nodes.
 * Modification: Do not calculate repulsion for port & process nodes
 */
//@Override
SbgnPDLayout.prototype.calcRepulsionForces = function ()
{
    var i, j;
    var nodeA, nodeB;
    var lNodes = this.getAllNodes();
    var processedNodeSet;
    
    if (this.useFRGridVariant)
    {
        // grid is a vector matrix that holds CoSENodes.
        // be sure to convert the Object type to CoSENode.
        if (this.totalIterations
                % SbgnPDConstants.GRID_CALCULATION_CHECK_PERIOD == 1)
        {
            this.grid = this.calcGrid(this.graphManager.getRoot());
            
            // put all nodes to proper grid cells
            for (i = 0; i < lNodes.length; i++)
            {
                nodeA = lNodes[i];
                this.addNodeToGrid(nodeA, 
                                   this.grid, 
                                   this.graphManager.getRoot().getLeft(), 
                                   this.graphManager.getRoot().getTop());
            }
        }

        processedNodeSet = new HashSet();

        // calculate repulsion forces between each nodes and its surrounding
        for (i = 0; i < lNodes.length; i++)
        {
            nodeA = lNodes[i];
            this.calculateRepulsionForceOfANode(this.grid, nodeA, processedNodeSet);
            processedNodeSet.add(nodeA);
        }
    }
    else
    {
        for (i = 0; i < lNodes.length; i++)
        {
            nodeA = lNodes[i];

            for (j = i + 1; j < lNodes.length; j++)
            {
                nodeB = lNodes[j];

                // If both nodes are not members of the same graph, skip.
                if (nodeA.getOwner() !== nodeB.getOwner())
                {
                    continue;
                }

                if (nodeA.type !== null && 
                    nodeB.type !== null && 
                    nodeA.getOwner() === nodeB.getOwner() && 
                    (nodeA.type === SbgnPDConstants.INPUT_PORT || 
                     nodeA.type === SbgnPDConstants.OUTPUT_PORT || 
                     nodeB.type === SbgnPDConstants.INPUT_PORT || 
                     nodeB.type === SbgnPDConstants.OUTPUT_PORT))
                {
                    continue;
                }

                this.calcRepulsionForce(nodeA, nodeB);
            }
        }
    }
};

/**
 * This method finds surrounding nodes of nodeA in repulsion range.
 * And calculates the repulsion forces between nodeA and its surrounding.
 * During the calculation, ignores the nodes that have already been processed.
 * Modification: Do not calculate repulsion for port & process nodes
 */
// @Override
SbgnPDLayout.prototype.calculateRepulsionForceOfANode = function (grid, nodeA, processedNodeSet)
{
    var i, j;

    if (this.totalIterations % FDLayoutConstants.GRID_CALCULATION_CHECK_PERIOD == 1)
    {
        var surrounding = new HashSet();
        var nodeB;

        for (i = (nodeA.startX - 1); i < (nodeA.finishX + 2); i++)
        {
            for (j = (nodeA.startY - 1); j < (nodeA.finishY + 2); j++)
            {
                if (!((i < 0) || (j < 0) || (i >= grid.length) || (j >= grid[0].length)))
                {
                    var numOfNodes = grid[i][j].length;
                    for (var k = 0; k < numOfNodes; k++)
                    {
                        nodeB = grid[i][j][k];

                        // If both nodes are not members of the same graph,
                        // or both nodes are the same, skip.
                        if ((nodeA.getOwner() !== nodeB.getOwner()) || 
                            (nodeA === nodeB))
                        {
                            continue;
                        }

                        if (nodeA.type !== null && 
                            nodeB.type !== null && 
                            nodeA.getOwner() === nodeB.getOwner() && 
                            (nodeA.type === SbgnPDConstants.INPUT_PORT || 
                             nodeA.type === SbgnPDConstants.OUTPUT_PORT || 
                             nodeB.type === SbgnPDConstants.INPUT_PORT || 
                             nodeB.type === SbgnPDConstants.OUTPUT_PORT))
                        {
                            continue;
                        }

                        // check if the repulsion force between
                        // nodeA and nodeB has already been calculated
                        if (!processedNodeSet.contains(nodeB) && !surrounding.contains(nodeB))
                        {
                            var distanceX = Math.abs(nodeA.getCenterX()
                                            - nodeB.getCenterX())
                                            - ((nodeA.getWidth() / 2) + (nodeB
                                                            .getWidth() / 2));
                            var distanceY = Math.abs(nodeA.getCenterY()
                                            - nodeB.getCenterY())
                                            - ((nodeA.getHeight() / 2) + (nodeB
                                                            .getHeight() / 2));
                                                    
                            // if the distance between nodeA and nodeB
                            // is less then calculation range
                            if ((distanceX <= this.repulsionRange) && 
                                (distanceY <= this.repulsionRange))
                            {
                                // then add nodeB to surrounding of nodeA
                                surrounding.add(nodeB);
                            }
                        }
                    }
                }
            }
        }
        
        nodeA.surrounding = surrounding.set;
    }

    for (i = 0; i < nodeA.surrounding.length; i++)
    {
        this.calcRepulsionForce(nodeA, nodeA.surrounding[i]);
    }
};

/**
* This method creates a port node with the associated type (input/output
* port)
*/
SbgnPDLayout.prototype.newPortNode = function (vNode, type)
{
    var n = new SbgnPDNode(this.graphManager, null, null, vNode, null);
    n.type = type;
    n.setWidth(SbgnPDConstants.PORT_NODE_DEFAULT_WIDTH);
    n.setHeight(SbgnPDConstants.PORT_NODE_DEFAULT_HEIGHT);

    return n;
};

/**
* This method creates an SBGNProcessNode object
*/
SbgnPDLayout.prototype.newProcessNode = function (vNode)
{
    return new SbgnProcessNode(this.graphManager, null, null, vNode);
};

/**
* This method creates a rigid edge.
*/
SbgnPDLayout.prototype.newRigidEdge = function (vEdge)
{
    var e = new SbgnPDEdge(null, null, vEdge);
    e.type = SbgnPDConstants.RIGID_EDGE;
    return e;
};

module.exports = SbgnPDLayout;

},{"./CoSELayout":5,"./Compaction":7,"./HashMap":13,"./HashSet":14,"./IGeometry":15,"./Integer":17,"./MemberPack":25,"./PointD":28,"./RectProc":33,"./RectangleD":34,"./SbgnPDConstants":35,"./SbgnPDEdge":36,"./SbgnPDNode":38,"./SbgnProcessNode":39}],38:[function(_dereq_,module,exports){
var CoSENode = _dereq_('./CoSENode');
var SbgnPDConstants = _dereq_('./SbgnPDConstants');
var PointD = _dereq_('./PointD');

// TODO: There is another contructor available in java, do we need it?

function SbgnPDNode(gm, loc, size, vNode, type) 
{
    CoSENode.call(this, gm, loc, size, vNode);
    
    this.type = type;
    this.visited = false;
    
    this.label = vNode ? vNode.label : null;
    this.isDummyCompound = false;
}

SbgnPDNode.prototype = Object.create(CoSENode.prototype);
for (var prop in CoSENode) {
  SbgnPDNode[prop] = CoSENode[prop];
}

SbgnPDNode.prototype.copy = function (/*SbgnPDNode*/ node) 
{
    this.type = node.type;
    this.label = node.label;
    this.setCenter(node.getCenterX(), node.getCenterY());
    this.setChild(node.getChild());
    this.setHeight(node.getHeight());
    this.setLocation(node.getLocation().x, node.getLocation().y);
    this.setNext(node.getNext());
    this.setOwner(node.getOwner());
    this.setPred1(node.getPred1());
    this.setPred2(node.getPred2());
    this.setWidth(node.getWidth());
};

SbgnPDNode.prototype.getSpringForceX = function ()
{
    return this.springForceX;
};

SbgnPDNode.prototype.isComplex = function ()
{
    return this.type.localeCompare(SbgnPDConstants.COMPLEX) === 0;
};

SbgnPDNode.prototype.isInputPort = function ()
{
    return this.type.localeCompare(SbgnPDConstants.INPUT_PORT) === 0;
};

SbgnPDNode.prototype.isOutputPort = function ()
{
    return this.type.localeCompare(SbgnPDConstants.OUTPUT_PORT) === 0;
};

/**
 * This method checks if the given node contains any unmarked complex nodes
 * in its child graph.
*/
SbgnPDNode.prototype.containsUnmarkedComplex = function ()
{
    if (this.getChild() == null)
    {
        return false;
    }
    else
    {
        var numOfChildren = this.getChild().getNodes().length;
        for (var index=0; index<numOfChildren; index++)
        {
            var child = this.getChild().getNodes()[index];
            if (child.isComplex() && !child.visited)
            {
                return true;
            }
        }
        return false;
    }
};

SbgnPDNode.prototype.resetForces = function ()
{
    this.springForceX = 0;
    this.springForceY = 0;
    this.repulsionForceX = 0;
    this.repulsionForceY = 0;
};

SbgnPDNode.prototype.rotateNode = function (/*PointD*/ origin, /*int*/ rotationDegree)
{
    var relativePt = new PointD(
            (this.getCenterX() - origin.x), (this.getCenterY() - origin.y));
    var rotatedPt = new PointD(
            (-Math.signum(rotationDegree) * relativePt.y), 
            (Math.signum(rotationDegree) * relativePt.x));

    this.setCenter(rotatedPt.x + origin.x, rotatedPt.y + origin.y);

    var newHeight = this.getWidth();
    var newWidth = this.getHeight();
    this.setWidth(newWidth);
    this.setHeight(newHeight);
};

/**
 * This method is used for port nodes only
 */
SbgnPDNode.prototype.calcAveragePoint = function ()
{
    var averagePnt = new PointD(0.0, 0.0);
    
    var numOfEdges = this.getEdges().length;
    for (var index=0; index<numOfEdges; index++)
    {
        var edge = this.getEdges()[index];
        
        if (edge.type.localeCompare(SbgnPDConstants.RIGID_EDGE) === 0)
        {
            continue;
        }
        
        averagePnt.x += edge.getOtherEnd(this).getCenterX();
        averagePnt.y += edge.getOtherEnd(this).getCenterY();
    }

    averagePnt.x /= (this.getEdges().length - 1);
    averagePnt.y /= (this.getEdges().length - 1);

    return averagePnt;
};

/**
* This method returns the neighbors of a given node. Notice that the graph
* is directed. Therefore edges should have the given node as the source
* node.
*/
SbgnPDNode.prototype.getChildrenNeighbors = function (/*String*/ edgeType)
{
    var neighbors = [];

    for (var index = 0; index<this.getEdges().length; index++)
    {
        var edge = this.getEdges()[index];

        if ((edge.getSource()== this) && (edge.getTarget() != this))
        {
            var node = edge.getTarget();

            if (edgeType != null && edge.type.localeCompare(edgeType) === 0)
            {
                neighbors.push(node);
            }
            if (edgeType == null)
            {
                neighbors.push(node);
            }
        }
    }
    return neighbors;
};

/**
 * This method updates the bounds of this compound node. If the node is a
 * dummy compound, do not include label and extra margins.
 */
SbgnPDNode.prototype.updateBounds = function ()
{
    // TODO: Do we need handle assertions?
    /*assert this.getChild() != null;*/

    if (this.getChild().getNodes().length !== 0)
    {
        // wrap the children nodes by re-arranging the boundaries
        var childGraph = this.getChild();
        childGraph.updateBounds(true);

        this.rect.x = childGraph.getLeft();
        this.rect.y = childGraph.getTop();

        if ((this.type != null) && 
            (this.type.localeCompare(SbgnPDConstants.DUMMY_COMPOUND) === 0))
        {
            this.setWidth(childGraph.getRight() - childGraph.getLeft());
            this.setHeight(childGraph.getBottom() - childGraph.getTop());
        }
        else
        {
            this.setWidth(childGraph.getRight() - childGraph.getLeft() + 2
                            * SbgnPDConstants.COMPOUND_NODE_MARGIN);
            this.setHeight(childGraph.getBottom() - childGraph.getTop() + 2
                            * SbgnPDConstants.COMPOUND_NODE_MARGIN
                            + SbgnPDConstants.LABEL_HEIGHT);
        }
    }
};
        
module.exports = SbgnPDNode;

},{"./CoSENode":6,"./PointD":28,"./SbgnPDConstants":35}],39:[function(_dereq_,module,exports){
var IGeometry = _dereq_('./IGeometry');
var PointD = _dereq_('./PointD');

var SbgnPDNode = _dereq_('./SbgnPDNode');
var SbgnPDEdge = _dereq_('./SbgnPDEdge');
var SbgnPDConstants = _dereq_('./SbgnPDConstants');

SbgnProcessNode.prototype.OrientationEnum = 
{
    BOTTOM_TO_TOP : 0, 
    TOP_TO_BOTTOM : 1,
    LEFT_TO_RIGHT : 2,
    RIGHT_TO_LEFT : 3
};

SbgnProcessNode.prototype.RotationPriorityEnum = 
{
    NINETY_DEGREE : 0, 
    SWAP : 1,
    NO_ROTATION : 2
};

function SbgnProcessNode(gm, loc, size, vNode, type) 
{
    SbgnPDNode.call(this, gm, loc, size, vNode, type);

    this.netRotationalForce = 0;
    this.consumptionEdges = [];
    this.productEdges = [];
    this.effectorEdges = [];
    
    this.parentCompound = null;
    this.inputPort = null;
    this.outputPort = null;
    
    this.orientation = null;
    this.rotationPriority = null;
    
    this.idealEdgeLength = 0.0;
    this.netRotationalForce = 0.0;
    this.properEdgeCount = 0.0;
}

SbgnProcessNode.prototype = Object.create(SbgnPDNode.prototype);
for (var prop in SbgnPDNode) {
  SbgnProcessNode[prop] = SbgnPDNode[prop];
}

/**
* Connect the port node to its process node (parent) and connect the edges
* of neighbor nodes to the port node by considering their types (for both
* input port and output port)
*/
SbgnProcessNode.prototype.reconnectEdges = function (idealEdgeLength)
{
    this.idealEdgeLength = idealEdgeLength;
    
    // change connections from process node&neighbors to port&neighbors.
    for (var i = 0; i < this.getEdges().length; i++)
    {
        var sEdge = this.getEdges()[i];

        if (sEdge.type === SbgnPDConstants.CONSUMPTION)
        {
            this.getEdges().splice(this.getEdges().indexOf(sEdge), 1);

            sEdge.setTarget(this.inputPort);
            this.inputPort.getEdges().push(sEdge);
            i--;
        }
        else if (sEdge.type === SbgnPDConstants.PRODUCTION)
        {
            this.getEdges().splice(this.getEdges().indexOf(sEdge), 1);

            sEdge.setSource(this.outputPort);
            this.outputPort.getEdges().push(sEdge);
            i--;
        }
        else if (sEdge.isEffector())
        {
            this.getEdges().splice(this.getEdges().indexOf(sEdge), 1);

            sEdge.setTarget(this.parentCompound);
            this.parentCompound.getEdges().push(sEdge);
            i--;
        }
    }
};

SbgnProcessNode.prototype.connectNodes = function (parentCompound, inputPort, outputPort)
{
    this.parentCompound = parentCompound;
    this.parentCompound.isDummyCompound = true;
    this.inputPort = inputPort;
    this.outputPort = outputPort;
    this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;

    // initial placement. place input to the left of the process node,
    // output to the right
    outputPort.setCenter(this.getCenterX() + SbgnPDConstants.RIGID_EDGE_LENGTH, 
                         this.getCenterY());
    inputPort.setCenter(this.getCenterX() - SbgnPDConstants.RIGID_EDGE_LENGTH, 
                        this.getCenterY());
};

/**
* Check if the process is eligible for rotation. First check if a
* 180-degree is possible (as it is more critical).
*/
SbgnProcessNode.prototype.isRotationNecessary = function ()
{
    // normalize the amount (per iteration)
    this.netRotationalForce /= (SbgnPDConstants.ROTATIONAL_FORCE_ITERATION_COUNT * 
                                 (this.consumptionEdges.length + 
                                  this.productEdges.length + 
                                  this.effectorEdges.length));

    if (this.isSwapAvailable())
    {
         this.rotationPriority = this.RotationPriorityEnum.SWAP;
         return true;
    }
    else if (Math.abs(this.netRotationalForce) > SbgnPDConstants.ROTATION_90_DEGREE)
    {
         this.rotationPriority = this.RotationPriorityEnum.NINETY_DEGREE;
         return true;
    }
    else
    {
         this.rotationPriority = this.RotationPriorityEnum.NO_ROTATION;
         return false;
    }
};

/**
* If the percentage of obtuse angles exceeds the threshold, swap is
* required.
*/
SbgnProcessNode.prototype.isSwapAvailable = function ()
{
    var obtuseAngleCnt = 0.0;
    var acuteAngleCnt = 0.0;

    var numOfConsumptionEdges = this.consumptionEdges.length;
    for (var i=0; i<numOfConsumptionEdges; i++)
    {
        var edge = this.consumptionEdges[i];
        
        if (Math.abs(edge.correspondingAngle) > 90)
            obtuseAngleCnt++;
        else
            acuteAngleCnt++;
    }
    
    var numOfProductEdges = this.productEdges.length;
    for (var i=0; i<numOfProductEdges; i++)
    {
        var edge = this.productEdges[i];
        
        if (Math.abs(edge.correspondingAngle) > 90)
                obtuseAngleCnt++;
        else
                acuteAngleCnt++;
    }

    if (obtuseAngleCnt / (obtuseAngleCnt + acuteAngleCnt) > SbgnPDConstants.ROTATION_180_DEGREE)
        return true;
    else
        return false;
};

/**
* This method rotates the associated compound (and its children: process
* and ports).
*/
SbgnProcessNode.prototype.applyRotation = function ()
{
    if (this.rotationPriority === this.RotationPriorityEnum.NINETY_DEGREE)
    {
        if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM)
        {
            if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation = this.OrientationEnum.RIGHT_TO_LEFT;
            }
            else if (netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;
            }
        }
        else if (this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
        {
            if (this.netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;
            }
            else if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation = this.OrientationEnum.RIGHT_TO_LEFT;
            }
        }
        else if (this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
        {
            if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation = this.OrientationEnum.BOTTOM_TO_TOP;
            }
            else if (this.netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation =  this.OrientationEnum.TOP_TO_BOTTOM;
            }
        }
        else if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT)
        {
            if (this.netRotationalForce < -SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(-90);
                this.orientation = this.OrientationEnum.BOTTOM_TO_TOP;
            }
            else if (this.netRotationalForce > SbgnPDConstants.ROTATION_90_DEGREE)
            {
                this.rotateCompound(90);
                this.orientation =  this.OrientationEnum.TOP_TO_BOTTOM;
            }
        }
    }

    else if (this.rotationPriority === this.RotationPriorityEnum.SWAP)
    {
        var tempCenter = this.inputPort.getCenter();
        
        this.inputPort.setCenter(this.outputPort.getCenterX(), 
                            this.outputPort.getCenterY());
        this.outputPort.setCenter(tempCenter.x, tempCenter.y);

        if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM)
            this.orientation = this.OrientationEnum.BOTTOM_TO_TOP;
        else if (this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
            this.orientation = this.OrientationEnum.TOP_TO_BOTTOM;
        else if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT)
            this.orientation = this.OrientationEnum.RIGHT_TO_LEFT;
        else if (this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
            this.orientation = this.OrientationEnum.LEFT_TO_RIGHT;
    }

    this.netRotationalForce = 0;
};


/**
* Given a compound node, this method recursively rotates the compound node
* and its members.
*/
SbgnProcessNode.prototype.rotateCompound = function (rotationDegree)
{
    this.rotateNode(this.getCenter(), rotationDegree);
    this.inputPort.rotateNode(this.getCenter(), rotationDegree);
    this.outputPort.rotateNode(this.getCenter(), rotationDegree);
    this.parentCompound.updateBounds();
};

SbgnProcessNode.prototype.calcRotationalForces = function ()
{
    this.netRotationalForce += this.calcProperlyOrientedEdges();
};

/**
* This method calculates all angles between process and its edges (prod,
* cons, eff) and marks them as properly oriented or not. Returned value is
* the amount of desire to rotate at this step. The returned value should be
* then manually added to netRotationalForce (if aim is to calculate
* netrotationalforce)
*/
SbgnProcessNode.prototype.calcProperlyOrientedEdges = function ()
{
    var inputRotSum = 0;
    var outputRotSum = 0;
    var effectorRotSum = 0;
    var stepSum = 0;
    var result;
    this.properEdgeCount = 0;

    // if the neighbors of port nodes have not been detected yet, find them.
    if (this.consumptionEdges.length === 0 && this.productEdges.length === 0)
    {
        this.initLists();
    }

    // find ideal positions
    var inputPortTarget = this.findPortTargetPoint(true, this.orientation);
    var outputPortTarget = this.findPortTargetPoint(false, this.orientation);

    for (var nodeIndex = 0; nodeIndex < this.consumptionEdges.length; nodeIndex++)
    {
        result = this.calcRotationalForce(true, nodeIndex, inputPortTarget);
        if (Math.abs(result) <= SbgnPDConstants.ANGLE_TOLERANCE)
        {
            this.properEdgeCount++;
        }
        inputRotSum += result;
    }
    for (var nodeIndex = 0; nodeIndex < this.productEdges.length; nodeIndex++)
    {
        result = this.calcRotationalForce(false, nodeIndex, outputPortTarget);
        if (Math.abs(result) <= SbgnPDConstants.ANGLE_TOLERANCE)
        {
            this.properEdgeCount++;
        }
        outputRotSum += result;
    }

    for (var nodeIndex = 0; nodeIndex < this.effectorEdges.length; nodeIndex++)
    {
        result = this.calcEffectorAngle(nodeIndex);
        if (Math.abs(result) <= SbgnPDConstants.EFFECTOR_ANGLE_TOLERANCE)
                this.properEdgeCount++;

        effectorRotSum += Math.abs(result);
    }

    // add total effector rotational force with the same sign of
    // step sum because it does not matter for an effector node
    // either rotate to left or right. therefore support the rotation
    // direction of each iteration.
    stepSum = inputRotSum - outputRotSum;
    stepSum = stepSum + (Math.sign(stepSum) * Math.abs(effectorRotSum));

    return stepSum;
};

/**
* This method returns the signed angle between a node and its corresponding
* port and the target point.
*/
SbgnProcessNode.prototype.calcRotationalForce = function (isInputPort, nodeIndex, targetPoint)
{
    var node;
    var centerPoint;

    if (isInputPort)
    {
        node = this.consumptionEdges[nodeIndex].getSource();
        centerPoint = this.inputPort.getCenter();
    }
    else
    {
        node = this.productEdges[nodeIndex].getTarget();
        centerPoint = this.outputPort.getCenter();
    }

    var angle = IGeometry.calculateAngle(targetPoint, centerPoint, node.getCenter());

    if (isInputPort)
        angle *= this.isLeft(targetPoint, centerPoint, node.getCenter(), SbgnPDConstants.INPUT_PORT);
    else
        angle *= this.isLeft(targetPoint, centerPoint, node.getCenter(), SbgnPDConstants.OUTPUT_PORT);

    this.saveInformation(isInputPort, nodeIndex, angle);

    return angle;
};

/**
* Calculates the angle between an effector edge and its process node. An
* effector edge has process (in this case the dummy compound) as its target
* node and the effector itself as the source.
*/
SbgnProcessNode.prototype.calcEffectorAngle = function (nodeIndex)
{
    var eff = this.effectorEdges[nodeIndex].getSource();
    var targetPnt = new PointD();
    var centerPnt = this.getCenter();

    // find target point
    if (this.isHorizontal())
    {
        targetPnt.x = this.getCenterX();

        if (eff.getCenterY() > this.getCenterY())
            targetPnt.y = this.getCenterY() + this.idealEdgeLength;
        else
            targetPnt.y = this.getCenterY() - this.idealEdgeLength;
    }
    else if (this.isVertical())
    {
        targetPnt.y = this.getCenterY();

        if (eff.getCenterX() > this.getCenterX())
                targetPnt.x = this.getCenterX() + this.idealEdgeLength;
        else
                targetPnt.x = this.getCenterX() - this.idealEdgeLength;
    }

    var angle = IGeometry.calculateAngle(targetPnt, centerPnt, eff.getCenter());

    this.effectorEdges[nodeIndex].correspondingAngle = angle;

    if (Math.abs(angle) <= SbgnPDConstants.EFFECTOR_ANGLE_TOLERANCE)
        this.effectorEdges[nodeIndex].isProperlyOriented = true;
    else
        this.effectorEdges[nodeIndex].isProperlyOriented = false;

    return angle;
};

SbgnProcessNode.prototype.applyApproximations = function ()
{
    // if there is only one single-edge consumption, move it to ideal
    // otherwise move towards multiedge node

    if (this.consumptionEdges.length === 1 && 
        this.consumptionEdges[0].getSource().getEdges().length === 1)
    {
        this.approximateForSingleNodes(this.inputPort, this.consumptionEdges[0].getSource());
    }
    else
    {
        this.approximateForMultipleNodes(this.inputPort);
    }

    if (this.productEdges.length === 1 && 
        this.productEdges[0].getTarget().getEdges().length === 1)
    {    
        this.approximateForSingleNodes(this.outputPort, this.productEdges[0].getTarget());
    }
    else
    {
        approximateForMultipleNodes(this.outputPort);
    }

    this.approximateEffectors();
};

SbgnProcessNode.prototype.approximateForSingleNodes = function (port, node)
{
    var targetPt = new PointD();
    var newPoint = new PointD();
    
    if (port.isInputPort())
    {
        targetPt = this.findPortTargetPoint(true, this.orientation);
    }    
    
    else if (port.isOutputPort())
    {
        targetPt = this.findPortTargetPoint(false, this.orientation);   
    }
    
    newPoint.x = targetPt.x
                    + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                    - SbgnPDConstants.APPROXIMATION_DISTANCE;
    newPoint.y = targetPt.y
                    + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                    - SbgnPDConstants.APPROXIMATION_DISTANCE;

    node.setCenter(newPoint.x, newPoint.y);
};

/**
* Given the port node, this method finds all consumption(or production)
* nodes of this port node. The idea is to move each single-edge
* consumptions(products) of a process closer to neighbor multi-edge nodes
* to keep them close to each other.
*/
SbgnProcessNode.prototype.approximateForMultipleNodes = function (port)
{
    var oneEdgeNodes = [];
    var multiEdgeNodes = [];
    var nodeOfInterest = null;
    var targetPt = new PointD();

    // get all non-rigid edges of port node
    var numOfEdges = port.getEdges().length;
    for (var i=0; i<numOfEdges; i++)
    {
        var edge = port.getEdges()[i];

        if (edge.isRigidEdge())
        {
            continue;
        }

        // node of interest depends on the direction of the edge
        if (port.isInputPort())
        {
            nodeOfInterest = edge.getSource();   
        }        
        else if (port.isOutputPort())
        {
            nodeOfInterest = edge.getTarget();
        }

        if (nodeOfInterest.getEdges().length === 1) // single node    
        {
            oneEdgeNodes.push(nodeOfInterest);
        }
        else if (nodeOfInterest.getEdges().length > 1) // multiedge node
        {
            multiEdgeNodes.push(nodeOfInterest);
        }
    }

    if (port.isInputPort())
    {
        targetPt = this.findPortTargetPoint(true, this.orientation);   
    }        
    else if (port.isOutputPort())
    {
        targetPt = this.findPortTargetPoint(false, this.orientation);
    }        
    
    // move
    if (oneEdgeNodes.length > 0)
    {
        moveOneEdgeNodes(oneEdgeNodes, multiEdgeNodes, targetPt);
    }    
};

/**
* Single-edge nodes are moved around the center point of a multi-edge node.
* If all the neighbor nodes of that port node are single-edged, then one of
* them is chosen randomly and the others are placed around it.
* 
* @param targetPt
*/
SbgnProcessNode.prototype.moveOneEdgeNodes = function (oneEdgeNodes, multiEdgeNodes, targetPt)
{
    var approximationPnt = new PointD(0, 0);
    var randomIndex = -1;
    var approximationNode = null;
    var newPoint = new PointD();

    // if there are more than one multi edge nodes, select the highly
    // connected one
    if (multiEdgeNodes.length > 0)
    {
        approximationNode = multiEdgeNodes[0];
        
        var numOfMultiEdgeNodes = multiEdgeNodes.length;
        for (var i=0; i<numOfMultiEdgeNodes; i++)
        {
            var node = multiEdgeNodes[i];

            if (node.getEdges().length > approximationNode.getEdges().length)
            {
                approximationNode = node;
            }
        }
    }

    // if there are no multi edge nodes, randomly select one
    else if (multiEdgeNodes.length === 0)
    {
        randomIndex = (Math.random() * oneEdgeNodes.length);
        approximationNode = oneEdgeNodes[randomIndex];
    }

    approximationPnt = approximationNode.getCenter();

    // move single nodes around the approximation point
    var numOfOneEdgeNodes = oneEdgeNodes.length;
    for (var i=0; i<numOfOneEdgeNodes; i++)
    {
        var s = oneEdgeNodes[i];
        
        // if they belong to different graphs, do not move
        // if (approximationNode.getOwner() != s.getOwner())
        // continue;
        newPoint.x = approximationPnt.x
                        + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                        - SbgnPDConstants.APPROXIMATION_DISTANCE;
        newPoint.y = approximationPnt.y
                        + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                        - SbgnPDConstants.APPROXIMATION_DISTANCE;

        s.setCenter(newPoint.x, newPoint.y);
    }
};

/**
* Identify single-edge effectors. Note that a process may have a number of
* effectors. Find the location of each effector. If the orientation of
* process node is vertical, ideal position of effectors is on the
* horizontal directions. (or vice versa)
*/
SbgnProcessNode.prototype.approximateEffectors = function ()
{
    var newPoint = new PointD();
    var approximationPnt = new PointD();

    // identify the effectors
    var numOfEffectorEdges = this.effectorEdges.length;
    for (var i=0; i<numOfEffectorEdges; i++)
    {
        var edge = this.effectorEdges[i];
        
        // source node of each effector edge is the effector itself. only
        // move single effectors
        if (edge.getSource().getEdges().length !== 1)
        {
            continue;
        }

        approximationPnt = this.findEffectorTargetPoint(edge.getSource());

        // place effector in a circular area using some randomness
        newPoint.x = approximationPnt.x
                        + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                        - SbgnPDConstants.APPROXIMATION_DISTANCE;
        newPoint.y = approximationPnt.y
                        + (Math.random() * SbgnPDConstants.APPROXIMATION_DISTANCE * 2)
                        - SbgnPDConstants.APPROXIMATION_DISTANCE;

        edge.getSource().setCenter(newPoint.x, newPoint.y);
    }
};

/**
* Given the effector and its corresponding process, the method returns the
* ideal position of the effector node, which has a distance of ideal edge
* length from its process.
*/
SbgnProcessNode.prototype.findEffectorTargetPoint = function (eff)
{
    var approximationPnt = new PointD();
    
    // find target point
    if (this.isHorizontal())
    {
        approximationPnt.x = this.getCenterX();

        if (eff.getCenterY() > this.getCenterY())
        {
            approximationPnt.y = this.getCenterY() + this.idealEdgeLength;
        }
        else
        {
            approximationPnt.y = this.getCenterY() - this.idealEdgeLength;
        }
    }
    else if (this.isVertical())
    {
        approximationPnt.y = this.getCenterY();

        if (eff.getCenterX() > this.getCenterX())
        {
            approximationPnt.x = this.getCenterX() + this.idealEdgeLength;
        }
        else
        {
            approximationPnt.x = this.getCenterX() - this.idealEdgeLength;
        }
    }

    return approximationPnt;
};

SbgnProcessNode.prototype.findPortTargetPoint = function (isInputPort, orient)
{
    if (orient === this.OrientationEnum.LEFT_TO_RIGHT)
    {
        if (isInputPort)
        {
            return new PointD((this.inputPort.getCenterX() - this.idealEdgeLength),
                              this.inputPort.getCenterY());
        }
        else
        {
            return new PointD((this.outputPort.getCenterX() + this.idealEdgeLength),
                              this.outputPort.getCenterY());
        }
    }
    else if (orient === this.OrientationEnum.RIGHT_TO_LEFT)
    {
        if (isInputPort)
        {
            return new PointD((this.inputPort.getCenterX() + this.idealEdgeLength),
                              this.inputPort.getCenterY());
        }
        else
        {
            return new PointD((this.outputPort.getCenterX() - this.idealEdgeLength),
                              this.outputPort.getCenterY());
        }
    }
    else if (orient === this.OrientationEnum.TOP_TO_BOTTOM)
    {
        if (isInputPort)
        {
            return new PointD(this.inputPort.getCenterX(),
                              (this.inputPort.getCenterY() - this.idealEdgeLength));
        }
        else
        {
            return new PointD(this.outputPort.getCenterX(),
                              (this.outputPort.getCenterY() + this.idealEdgeLength));
        }
    }
    else if (orient === this.OrientationEnum.BOTTOM_TO_TOP)
    {
        if (isInputPort)
        {
            return new PointD(this.inputPort.getCenterX(),
                              (this.inputPort.getCenterY() + this.idealEdgeLength));
        }
        else
        {
            return new PointD(this.outputPort.getCenterX(),
                              (this.outputPort.getCenterY() - this.idealEdgeLength));
        }
    }

    return null;
};

SbgnProcessNode.prototype.initLists = function ()
{
    var numOfConsumptionEdges = this.inputPort.getEdges().length;
    for (var i=0; i<numOfConsumptionEdges; i++)
    {
        var edge = this.inputPort.getEdges()[i];
        
        if (!edge.isRigidEdge())
        {
            this.consumptionEdges.add(edge);
        }
    }

    var numOfProductEdges = this.outputPort.getEdges().length;
    for (var i=0; i<numOfProductEdges; i++)
    {
        var edge = this.outputPort.getEdges()[i];

        if (!edge.isRigidEdge())
        {
            this.productEdges.add(edge);
        }
    }

    // detect all effector nodes connected to this process node (if
    // there are any)
    var numOfEffectorEdges = this.parentCompound.getEdges().length;
    for (var i=0; i<numOfEffectorEdges; i++)
    {
        var edge = this.parentCompound.getEdges()[i];

        if (edge.isEffector())
        {
            this.effectorEdges.add(edge);
        }
    }
};

SbgnProcessNode.prototype.saveInformation = function (isInputPort, nodeIndex, angle)
{

    // remember angles especially for debug purposes
    if (isInputPort)
    {
        this.consumptionEdges[nodeIndex].correspondingAngle = angle;
    }
    else
    {
        this.productEdges[nodeIndex].correspondingAngle = angle;
    }

    // note if the edges are properly oriented
    if (Math.abs(angle) <= SbgnPDConstants.ANGLE_TOLERANCE)
    {
        if (isInputPort)
        {
            this.consumptionEdges[nodeIndex].isProperlyOriented = true;
        }
        else
        {
            this.productEdges[nodeIndex].isProperlyOriented = true;
        }
    }
    else
    {
        if (isInputPort)
        {
            this.consumptionEdges[nodeIndex].isProperlyOriented = false;
        }
        else
        {
            this.productEdges[nodeIndex].isProperlyOriented = false;
        }
    }
};

SbgnProcessNode.prototype.isLeft = function (a, b, c, type)
{
    if (((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) > 0)
    {
        // left turn
        if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return -1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return 1;
        }
        else if (this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return 1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return -1;
        }
        else if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return 1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return -1;
        }
        else if (this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return -1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return 1;
        }
    }
    else
    {
        // right turn
        if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return 1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return -1;
        }
        else if (this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return -1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return 1;
        }
        else if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return -1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return 1;
        }
        else if (this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
        {
            if (type === SbgnPDConstants.INPUT_PORT)
                return 1;
            else if (type === SbgnPDConstants.OUTPUT_PORT)
                return -1;
        }
    }
    return 0;
};

SbgnProcessNode.prototype.copyNode = function (s, graphManager) // TODO: graphManager is not used.
{
    this.type = s.type;
    this.label = s.label;
    this.parentCompound = s.parentCompound;
    this.inputPort = s.inputPort;
    this.outputPort = s.outputPort;
    this.setCenter(s.getCenterX(), s.getCenterY());
    this.setChild(s.getChild());
    this.setHeight(s.getHeight());
    this.setLocation(s.getLocation().x, s.getLocation().y);
    this.setNext(s.getNext());
    this.setOwner(s.getOwner());
    this.setPred1(s.getPred1());
    this.setPred2(s.getPred2());
    this.setWidth(s.getWidth());
};

SbgnProcessNode.prototype.copyFromSBGNPDNode = function (s, graphManager)
{
    this.type = s.type;
    this.label = s.label;
    this.setCenter(s.getCenterX(), s.getCenterY());
    this.setChild(s.getChild());
    this.setHeight(s.getHeight());
    this.setLocation(s.getLocation().x, s.getLocation().y);
    this.setNext(s.getNext());
    this.setOwner(s.getOwner());
    this.setPred1(s.getPred1());
    this.setPred2(s.getPred2());
    this.setWidth(s.getWidth());

    // copy edges
    var numOfEdges = s.getEdges().length;
    for (var i=0; i<numOfEdges; i++)
    {
        var edge = s.getEdges()[i];
        var newEdge = new SbgnPDEdge(edge.getSource(), edge.getTarget(), null, edge.type);

        newEdge.copy(edge);

        if (edge.getSource() === s)
        {
            newEdge.setSource(this);
        }
        else if (edge.getTarget() === s)
        {
            newEdge.setTarget(this);
        }

        // add new edge to the graph manager.
        graphManager.add(newEdge, newEdge.getSource(), newEdge.getTarget());
    }
};

SbgnProcessNode.prototype.isVertical = function ()
{
    if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM || 
        this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
    {
        return true;
    }
    else
    {
        return false;   
    }
    
};

SbgnProcessNode.prototype.isHorizontal = function ()
{
    if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT || 
        this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
    {
        return true;
    }
    else
    {
        return false;
    }
};

SbgnProcessNode.prototype.setOrientation = function (orient)
{
    this.orientation = orient;
    if (this.orientation === this.OrientationEnum.LEFT_TO_RIGHT)
    {
        this.inputPort.setCenter(this.getCenterX()
                        - SbgnPDConstants.RIGID_EDGE_LENGTH, this.getCenterY());
        this.outputPort.setCenter(this.getCenterX()
                        + SbgnPDConstants.RIGID_EDGE_LENGTH, this.getCenterY());
    }
    else if (this.orientation === this.OrientationEnum.RIGHT_TO_LEFT)
    {
        this.inputPort.setCenter(this.getCenterX()
                        + SbgnPDConstants.RIGID_EDGE_LENGTH, this.getCenterY());
        this.outputPort.setCenter(this.getCenterX()
                        - SbgnPDConstants.RIGID_EDGE_LENGTH, this.getCenterY());
    }
    else if (this.orientation === this.OrientationEnum.BOTTOM_TO_TOP)
    {
        this.inputPort.setCenter(this.getCenterX(), this.getCenterY()
                        + SbgnPDConstants.RIGID_EDGE_LENGTH);
        this.outputPort.setCenter(this.getCenterX(), this.getCenterY()
                        - SbgnPDConstants.RIGID_EDGE_LENGTH);
    }
    else if (this.orientation === this.OrientationEnum.TOP_TO_BOTTOM)
    {
        this.inputPort.setCenter(this.getCenterX(), this.getCenterY()
                        - SbgnPDConstants.RIGID_EDGE_LENGTH);
        this.outputPort.setCenter(this.getCenterX(), this.getCenterY()
                        + SbgnPDConstants.RIGID_EDGE_LENGTH);
    }
};

/**
* Transfer forces acting on process node to its parent compound to make
* sure process does not move.
*/
SbgnProcessNode.prototype.transferForces = function ()
{
    this.parentCompound.springForceX += this.springForceX
                    + this.inputPort.springForceX + this.outputPort.springForceX;
    this.parentCompound.springForceY += this.springForceY
                    + this.inputPort.springForceY + this.outputPort.springForceY;
};

SbgnProcessNode.prototype.getInputPort = function ()
{
    return this.inputPort;
};

SbgnProcessNode.prototype.getOutputPort = function ()
{
    return this.outputPort;
};

SbgnProcessNode.prototype.getParentCompound = function ()
{
    return parentCompound;
};

module.exports = SbgnProcessNode;

},{"./IGeometry":15,"./PointD":28,"./SbgnPDConstants":35,"./SbgnPDEdge":36,"./SbgnPDNode":38}],40:[function(_dereq_,module,exports){
function Transform(x, y) {
  this.lworldOrgX = 0.0;
  this.lworldOrgY = 0.0;
  this.ldeviceOrgX = 0.0;
  this.ldeviceOrgY = 0.0;
  this.lworldExtX = 1.0;
  this.lworldExtY = 1.0;
  this.ldeviceExtX = 1.0;
  this.ldeviceExtY = 1.0;
}

Transform.prototype.getWorldOrgX = function ()
{
  return this.lworldOrgX;
}

Transform.prototype.setWorldOrgX = function (wox)
{
  this.lworldOrgX = wox;
}

Transform.prototype.getWorldOrgY = function ()
{
  return this.lworldOrgY;
}

Transform.prototype.setWorldOrgY = function (woy)
{
  this.lworldOrgY = woy;
}

Transform.prototype.getWorldExtX = function ()
{
  return this.lworldExtX;
}

Transform.prototype.setWorldExtX = function (wex)
{
  this.lworldExtX = wex;
}

Transform.prototype.getWorldExtY = function ()
{
  return this.lworldExtY;
}

Transform.prototype.setWorldExtY = function (wey)
{
  this.lworldExtY = wey;
}

/* Device related */

Transform.prototype.getDeviceOrgX = function ()
{
  return this.ldeviceOrgX;
}

Transform.prototype.setDeviceOrgX = function (dox)
{
  this.ldeviceOrgX = dox;
}

Transform.prototype.getDeviceOrgY = function ()
{
  return this.ldeviceOrgY;
}

Transform.prototype.setDeviceOrgY = function (doy)
{
  this.ldeviceOrgY = doy;
}

Transform.prototype.getDeviceExtX = function ()
{
  return this.ldeviceExtX;
}

Transform.prototype.setDeviceExtX = function (dex)
{
  this.ldeviceExtX = dex;
}

Transform.prototype.getDeviceExtY = function ()
{
  return this.ldeviceExtY;
}

Transform.prototype.setDeviceExtY = function (dey)
{
  this.ldeviceExtY = dey;
}

Transform.prototype.transformX = function (x)
{
  var xDevice = 0.0;
  var worldExtX = this.lworldExtX;
  if (worldExtX != 0.0)
  {
    xDevice = this.ldeviceOrgX +
            ((x - this.lworldOrgX) * this.ldeviceExtX / worldExtX);
  }

  return xDevice;
}

Transform.prototype.transformY = function (y)
{
  var yDevice = 0.0;
  var worldExtY = this.lworldExtY;
  if (worldExtY != 0.0)
  {
    yDevice = this.ldeviceOrgY +
            ((y - this.lworldOrgY) * this.ldeviceExtY / worldExtY);
  }


  return yDevice;
}

Transform.prototype.inverseTransformX = function (x)
{
  var xWorld = 0.0;
  var deviceExtX = this.ldeviceExtX;
  if (deviceExtX != 0.0)
  {
    xWorld = this.lworldOrgX +
            ((x - this.ldeviceOrgX) * this.lworldExtX / deviceExtX);
  }


  return xWorld;
}

Transform.prototype.inverseTransformY = function (y)
{
  var yWorld = 0.0;
  var deviceExtY = this.ldeviceExtY;
  if (deviceExtY != 0.0)
  {
    yWorld = this.lworldOrgY +
            ((y - this.ldeviceOrgY) * this.lworldExtY / deviceExtY);
  }
  return yWorld;
}

Transform.prototype.inverseTransformPoint = function (inPoint)
{
  var outPoint =
          new PointD(this.inverseTransformX(inPoint.x),
                  this.inverseTransformY(inPoint.y));
  return outPoint;
}

module.exports = Transform;

},{}],41:[function(_dereq_,module,exports){
function UniqueIDGeneretor() {
}

UniqueIDGeneretor.lastID = 0;

UniqueIDGeneretor.createID = function (obj) {
  if (UniqueIDGeneretor.isPrimitive(obj)) {
    return obj;
  }
  if (obj.uniqueID != null) {
    return obj.uniqueID;
  }
  obj.uniqueID = UniqueIDGeneretor.getString();
  UniqueIDGeneretor.lastID++;
  return obj.uniqueID;
}

UniqueIDGeneretor.getString = function (id) {
  if (id == null)
    id = UniqueIDGeneretor.lastID;
  return "Object#" + id + "";
}

UniqueIDGeneretor.isPrimitive = function (arg) {
  var type = typeof arg;
  return arg == null || (type != "object" && type != "function");
}

module.exports = UniqueIDGeneretor;

},{}],42:[function(_dereq_,module,exports){
var CoSEEdge = _dereq_('./CoSEEdge');

function VisibilityEdge(source, target, vEdge) {
  CoSEEdge.call(this, source, target, vEdge);
}

VisibilityEdge.prototype = Object.create(CoSEEdge.prototype);
for (var prop in CoSEEdge) {
  VisibilityEdge[prop] = CoSEEdge[prop];
}

/**
* We want the length of a visibility edge calculated as the distance
* between borders of two nodes, not the distance between center points.
* Edges have to be vertical or horizontal.
*/
// @Override
VisibilityEdge.prototype.updateLength = function ()
{
    if (this.source.getBottom() <= this.target.getTop())
    {
        this.lengthX = 0;
        this.lengthY = this.source.getBottom() - this.target.getTop();
    } 
    else if (this.source.getRight() <= this.target.getLeft())
    {
        this.lengthX = this.source.getRight() - this.target.getLeft();
        this.lengthY = 0;
    }
    // else
    // System.out.println("unexpected edge");

    this.length = Math.sqrt((this.lengthX * this.lengthX) + (this.lengthY * this.lengthY));
};

module.exports = VisibilityEdge;

},{"./CoSEEdge":2}],43:[function(_dereq_,module,exports){
var Integer = _dereq_('./Integer');
var RectangleD = _dereq_('./RectangleD');

var LGraph = _dereq_('./LGraph');
var VisibilityEdge = _dereq_('./VisibilityEdge');
var Compaction = _dereq_('./Compaction');

function VisibilityGraph(parent, graphMgr, vGraph) 
{
    LGraph.call(this, parent, graphMgr, vGraph);
    
    this.direction = null;
}

VisibilityGraph.prototype = Object.create(LGraph.prototype);
for (var prop in LGraph) {
  VisibilityGraph[prop] = LGraph[prop];
}

/**
* Create a new visibility graph. Compare each vertices in the graph to see
* if they are visible to each other. If two vertices can see each other and
* they see each other in the desired direction, add an edge between them.
*/
VisibilityGraph.prototype.construct = function (d, vertices)
{
    this.init(vertices);

    var nodes = this.getNodes();
    this.direction = d;

    // check the visibility between each two vertex
    for (var i = 0; i < nodes.length; i++)
    {
        for (var j = i + 1; j < nodes.length; j++)
        {
            var node1 = nodes[i];
            var node2 = nodes[j];

            var result = this.findVisibilityDirection(node1, node2);

            // if they are visible, create edge.
            if (result !== 0)
            {
                this.createEdge(node1, node2);
            }
        }
    }
};

/**
* This method adds the given nodes to the graph.
*/
VisibilityGraph.prototype.init = function (vertices)
{
    // create the new graph with given vertices
    var numOfVertices = vertices.length;
    for (var i=0; i<numOfVertices; i++)
    {
        this.add(vertices[i]);
    }
};

/**
* Given two nodes, check their visibility. Two nodes are visible to each
* other if there exists an infinite ray that intersects them without
* intersecting any other nodes in between those two.
* 
* @return 1: vertical, 2: horizontal (if any edge found). Otherwise, return
*         0
*/
VisibilityGraph.prototype.findVisibilityDirection = function (p, q)
{
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        // ensure that p points to the leftmost element
        if (q.getLeft() < p.getLeft() && 
            p.getLeft() < (q.getLeft() + q.getWidth()))
        {
            var temp = p;
            p = q;
            q = temp;
        }

        // check if there exists a ray
        if (p.getLeft() <= q.getLeft() && 
            q.getLeft() <= (p.getLeft() + p.getWidth()))
        {
            if (this.sweepIntersectedArea(p, q))
            {
                return 1;
            }
        }
    }

    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        // ensure that p points to the upper element
        if (q.getTop() < p.getTop() && 
            p.getTop() < (q.getTop() + q.getHeight()))
        {
            var temp = p;
            p = q;
            q = temp;
        }

        // check if there exists a ray
        if (p.getTop() <= q.getTop() && 
            q.getTop() <= (p.getTop() + p.getHeight()))
        {
            if (this.sweepIntersectedArea(p, q))
            {
                return 2;
            }
        }
    }
    return 0;
};

/**
* Starting from the intersection area between p and q, walk on a line
* perpendicular to the desired direction. If there is an edge that does not
* intersect with any other nodes, this is a valid edge.
* 
* @return true if an edge exists. false otherwise.
*/
VisibilityGraph.prototype.sweepIntersectedArea = function (p, q)
{
    var edge;
    var isValid;
    var start = 0;
    var end = 0;
    var result;

    // find the sweep line borders
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        start = q.getLeft();
        end = Math.min(p.getRight(), q.getRight());
    }
    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        start = q.getTop();
        end = Math.min(p.getBottom(), q.getBottom());
    }

    // if they intersect only on the borders, immediately return false.
    if (start === end)
    {
        return false;
    }

    // check for all intersected area
    for (var sweepPoint = start; sweepPoint <= end; sweepPoint++)
    {
        isValid = true;
        edge = this.tryConstructingEdge(p, q, sweepPoint);

        // if an edge is constructed, check its validity
        if (edge !== null)
        {
            result = this.checkIntermediateNodes(p, q, edge, sweepPoint);

            if (sweepPoint === result)
                isValid = true;
            else
            {
                sweepPoint = result;
                isValid = false;
            }
        }
        if (isValid)
            return true;
    }

    return false;
};

/**
* This method tries to construct an edge(RectangleD shape) between two
* nodes. The parameter i indicates the starting point of the edge. For
* example, if a vertical edge to be constructed, i is the top coordinate.
* For horizontal, i is the leftmost coordinate.
* 
* @return edge. if no edge can be constructed, returns null.
*/
VisibilityGraph.prototype.tryConstructingEdge = function (p, q, i)
{
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        // create an edge from upper to lower or return false:does not
        // exist
        if (p.getTop() < q.getTop() && 
            p.getBottom() <= q.getTop())
        {
            return new RectangleD(i, p.getBottom(), 1, (q.getTop() - p.getBottom()));
        }
        else if (q.getTop() < p.getTop() && 
                 (q.getTop() + q.getHeight()) <= p.getTop())
        {
            return new RectangleD(i, q.getBottom(), 1, (p.getTop() - q.getBottom()));
        }
        else
        {
            return null;
        }
    }
    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        // create an edge from leftmost to right or return false:does
        // not exist
        if (p.getLeft() < q.getLeft() && 
            p.getRight() <= q.getLeft())
        {
            return new RectangleD(p.getRight(), i, (q.getLeft() - p.getRight()), 1);
        }
        else if (q.getLeft() < p.getLeft() && 
                 q.getRight() <= p.getLeft())
        {
            return new RectangleD(q.getRight(), i, (p.getLeft() - q.getRight()), 1);
        }
        else
        {
            return null;
        }
    }

    return null;
};

/**
* This method checks if the given edge intersects any nodes except the
* source and target nodes. If an intersection is found, update the sweep
* point to the end of the intersected node. Otherwise, do not change the
* point.
*/
VisibilityGraph.prototype.checkIntermediateNodes = function (p, q, edge, sweepPoint)
{
    for (var j = 0; j < this.getNodes().length; j++)
    {
        var intermediateNode = this.getNodes()[j];

        if (intermediateNode !== p && intermediateNode !== q)
        {
            // if there is an intersection, edge is not valid
            if (edge.intersects(intermediateNode.getRect()))
            {
                // jump to the end of intersected node
                if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
                    sweepPoint = (intermediateNode.getRight() + 1);
                else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
                    sweepPoint = (intermediateNode.getBottom() + 1);

                break;
            }
        }
    }

    return sweepPoint;
};

/**
* This class creates an edge between the given nodes using the given
* direction. While adding the edge, be careful about the source and target
* i.e. if we want to get a vertical visibility graph, (A -> B) A should
* have a lower y coordinate (upper). Similarly, for horizontal visibility
* graph (A -> B): A is on the left, has lower x coordinate.
*/
VisibilityGraph.prototype.createEdge = function (node1, node2)
{
    if (this.direction === Compaction.CompactionDirectionEnum.VERTICAL)
    {
        if (node1.getTop() < node2.getTop())
            this.add(new VisibilityEdge(node1, node2, null), node1, node2);
        else
            this.add(new VisibilityEdge(node2, node1, null), node2, node1);
    }

    else if (this.direction === Compaction.CompactionDirectionEnum.HORIZONTAL)
    {
        if (node1.getLeft() < node2.getLeft())
            this.add(new VisibilityEdge(node1, node2, null), node1, node2);
        else
            this.add(new VisibilityEdge(node2, node1, null), node2, node1);
    }

    // calculate newly added edge's length.
    this.getEdges()[this.getEdges().length - 1].updateLength();
};

/**
* For each edge having s as its target node, find and return the shortest
* one. Returns null if could not find an edge.
*/
VisibilityGraph.prototype.findShortestEdge = function (s)
{
    var shortestEdge = null;
    var minLength = Integer.MAX_VALUE;

    for (var i = 0; i < this.getEdges().length; i++)
    {
        var e = this.getEdges()[i];

        e.updateLength();
        if (e.getTarget() === s && e.getLength() < minLength)
        {
            shortestEdge = e;
            minLength = e.getLength();
        }
    }

    return shortestEdge;
};

module.exports = VisibilityGraph;

},{"./Compaction":7,"./Integer":17,"./LGraph":19,"./RectangleD":34,"./VisibilityEdge":42}],44:[function(_dereq_,module,exports){
'use strict';

var Thread;

var DimensionD = _dereq_('./DimensionD');
var HashMap = _dereq_('./HashMap');
var HashSet = _dereq_('./HashSet');
var IGeometry = _dereq_('./IGeometry');
var IMath = _dereq_('./IMath');
var Integer = _dereq_('./Integer');
var Point = _dereq_('./Point');
var PointD = _dereq_('./PointD');
var RandomSeed = _dereq_('./RandomSeed');
var RectangleD = _dereq_('./RectangleD');
var Transform = _dereq_('./Transform');
var UniqueIDGeneretor = _dereq_('./UniqueIDGeneretor');
var LGraphObject = _dereq_('./LGraphObject');
var LGraph = _dereq_('./LGraph');
var LEdge = _dereq_('./LEdge');
var LGraphManager = _dereq_('./LGraphManager');
var LNode = _dereq_('./LNode');
var Layout = _dereq_('./Layout');
var LayoutConstants = _dereq_('./LayoutConstants');
var FDLayout = _dereq_('./FDLayout');
var FDLayoutConstants = _dereq_('./FDLayoutConstants');
var FDLayoutEdge = _dereq_('./FDLayoutEdge');
var FDLayoutNode = _dereq_('./FDLayoutNode');
var CoSEConstants = _dereq_('./CoSEConstants');
var CoSEEdge = _dereq_('./CoSEEdge');
var CoSEGraph = _dereq_('./CoSEGraph');
var CoSEGraphManager = _dereq_('./CoSEGraphManager');
var CoSELayout = _dereq_('./CoSELayout');
var CoSENode = _dereq_('./CoSENode');
var Compaction = _dereq_('./Compaction');
var SbgnPDConstants = _dereq_('./SbgnPDConstants');
var SbgnPDEdge = _dereq_('./SbgnPDEdge');
var SbgnPDLayout = _dereq_('./SbgnPDLayout');
var SbgnPDNode = _dereq_('./SbgnPDNode');
var SbgnProcessNode = _dereq_('./SbgnProcessNode');
var VisibilityEdge = _dereq_('./VisibilityEdge');
var VisibilityGraph = _dereq_('./VisibilityGraph');
var MemberPack = _dereq_('./MemberPack');
var Organization = _dereq_('./Organization');
var PolyominoQuickSort = _dereq_('./PolyominoQuickSort');
var PolyominoPacking = _dereq_('./PolyominoPacking');
var RectProc = _dereq_('./RectProc');

_SbgnPDLayout.idToLNode = {};
_SbgnPDLayout.toBeTiled = {};

// TODO: Bunlar layout default'lari mi? Yoksa CoSE specific default'lar mi?

var defaults = {
    // Called on `layoutready`
    ready: function () {
    },
    // Called on `layoutstop`
    stop: function () {
    },
    // Whether to fit the network view after when done
    fit: true,
    // Padding on fit
    padding: 10,
    // Whether to enable incremental mode
    randomize: true,
    // Node repulsion (non overlapping) multiplier
    nodeRepulsion: 4500,
    // Ideal edge (non nested) length
    idealEdgeLength: 50,
    // Divisor to compute edge forces
    edgeElasticity: 0.45,
    // Nesting factor (multiplier) to compute ideal edge length for nested edges
    nestingFactor: 0.1,
    // Gravity force (constant)
    gravity: 0.25,
    // Maximum number of iterations to perform
    numIter: 2500,
    // For enabling tiling
    tile: true,
    // Type of layout animation. The option set is {'during', 'end', false}
    animate: 'end',
    // Represents the amount of the vertical space to put between the zero degree members during the tiling operation(can also be a function)
    tilingPaddingVertical: 10,
    // Represents the amount of the horizontal space to put between the zero degree members during the tiling operation(can also be a function)
    tilingPaddingHorizontal: 10,
    // Gravity range (constant) for compounds
    gravityRangeCompound: 1.5,
    // Gravity force (constant) for compounds
    gravityCompound: 1.0,
    // Gravity range (constant)
    gravityRange: 3.8
};

function extend(defaults, options) {
    var obj = {};

    for (var i in defaults) {
        obj[i] = defaults[i];
    }

    for (var i in options) {
        obj[i] = options[i];
    }

    return obj;
}
;

_SbgnPDLayout.layout = new SbgnPDLayout();
function _SbgnPDLayout(options) {

    this.options = extend(defaults, options);
    _SbgnPDLayout.getUserOptions(this.options);
}

_SbgnPDLayout.getUserOptions = function (options) 
{
    /** TODO: Do we need more constansts (SBGN specific) here? */
    
    if (options.nodeRepulsion != null)
        SbgnPDConstants.DEFAULT_REPULSION_STRENGTH = CoSEConstants.DEFAULT_REPULSION_STRENGTH = FDLayoutConstants.DEFAULT_REPULSION_STRENGTH = options.nodeRepulsion;
    if (options.idealEdgeLength != null)
        SbgnPDConstants.DEFAULT_EDGE_LENGTH = CoSEConstants.DEFAULT_EDGE_LENGTH = FDLayoutConstants.DEFAULT_EDGE_LENGTH = options.idealEdgeLength;
    if (options.edgeElasticity != null)
        SbgnPDConstants.DEFAULT_SPRING_STRENGTH = CoSEConstants.DEFAULT_SPRING_STRENGTH = FDLayoutConstants.DEFAULT_SPRING_STRENGTH = options.edgeElasticity;
    if (options.nestingFactor != null)
        SbgnPDConstants.PER_LEVEL_IDEAL_EDGE_LENGTH_FACTOR = CoSEConstants.PER_LEVEL_IDEAL_EDGE_LENGTH_FACTOR = FDLayoutConstants.PER_LEVEL_IDEAL_EDGE_LENGTH_FACTOR = options.nestingFactor;
    if (options.gravity != null)
        SbgnPDConstants.DEFAULT_GRAVITY_STRENGTH = CoSEConstants.DEFAULT_GRAVITY_STRENGTH = FDLayoutConstants.DEFAULT_GRAVITY_STRENGTH = options.gravity;
    if (options.numIter != null)
        SbgnPDConstants.MAX_ITERATIONS = CoSEConstants.MAX_ITERATIONS = FDLayoutConstants.MAX_ITERATIONS = options.numIter;
    if (options.gravityRange != null)
        SbgnPDConstants.DEFAULT_GRAVITY_RANGE_FACTOR = CoSEConstants.DEFAULT_GRAVITY_RANGE_FACTOR = FDLayoutConstants.DEFAULT_GRAVITY_RANGE_FACTOR = options.gravityRange;
    if (options.gravityCompound != null)
        SbgnPDConstants.DEFAULT_COMPOUND_GRAVITY_STRENGTH = CoSEConstants.DEFAULT_COMPOUND_GRAVITY_STRENGTH = FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_STRENGTH = options.gravityCompound;
    if (options.gravityRangeCompound != null)
        SbgnPDConstants.DEFAULT_COMPOUND_GRAVITY_RANGE_FACTOR = CoSEConstants.DEFAULT_COMPOUND_GRAVITY_RANGE_FACTOR = FDLayoutConstants.DEFAULT_COMPOUND_GRAVITY_RANGE_FACTOR = options.gravityRangeCompound;

    SbgnPDConstants.DEFAULT_INCREMENTAL = CoSEConstants.DEFAULT_INCREMENTAL = FDLayoutConstants.DEFAULT_INCREMENTAL = LayoutConstants.DEFAULT_INCREMENTAL =
            !(options.randomize);
    SbgnPDConstants.ANIMATE = CoSEConstants.ANIMATE = FDLayoutConstants.ANIMATE = options.animate;
};

_SbgnPDLayout.prototype.run = function () {
    var layout = this;

    _SbgnPDLayout.idToLNode = {};
    _SbgnPDLayout.toBeTiled = {};
    _SbgnPDLayout.layout = new SbgnPDLayout();
    this.cy = this.options.cy;
    var after = this;

    this.cy.trigger('layoutstart');

    var gm = _SbgnPDLayout.layout.newGraphManager();
    this.gm = gm;

    var nodes = this.options.eles.nodes();
    var edges = this.options.eles.edges();

    this.root = gm.addRoot();

//    if (!this.options.tile) {
        this.processChildrenList(this.root, _SbgnPDLayout.getTopMostNodes(nodes));
//    } else {
//        // Find zero degree nodes and create a compound for each level
//        var memberGroups = this.groupZeroDegreeMembers();
//        // Tile and clear children of each compound
//        var tiledMemberPack = this.clearCompounds(this.options);
//        // Separately tile and clear zero degree nodes for each level
//        var tiledZeroDegreeNodes = this.clearZeroDegreeMembers(memberGroups);
//    }


    for (var i = 0; i < edges.length; i++) {
        var edge = edges[i];
        var sourceNode = _SbgnPDLayout.idToLNode[edge.data("source")];
        var targetNode = _SbgnPDLayout.idToLNode[edge.data("target")];
        var e1 = gm.add(_SbgnPDLayout.layout.newEdge(), sourceNode, targetNode);
        e1.id = edge.id();
    }


    var t1 = layout.thread;

    if (!t1 || t1.stopped()) { // try to reuse threads
        t1 = layout.thread = Thread();

        t1.require(DimensionD, 'DimensionD');
        t1.require(HashMap, 'HashMap');
        t1.require(HashSet, 'HashSet');
        t1.require(IGeometry, 'IGeometry');
        t1.require(IMath, 'IMath');
        t1.require(Integer, 'Integer');
        t1.require(Point, 'Point');
        t1.require(PointD, 'PointD');
        t1.require(RandomSeed, 'RandomSeed');
        t1.require(RectangleD, 'RectangleD');
        t1.require(Transform, 'Transform');
        t1.require(UniqueIDGeneretor, 'UniqueIDGeneretor');
        t1.require(LGraphObject, 'LGraphObject');
        t1.require(LGraph, 'LGraph');
        t1.require(LEdge, 'LEdge');
        t1.require(LGraphManager, 'LGraphManager');
        t1.require(LNode, 'LNode');
        t1.require(Layout, 'Layout');
        t1.require(LayoutConstants, 'LayoutConstants');
        t1.require(FDLayout, 'FDLayout');
        t1.require(FDLayoutConstants, 'FDLayoutConstants');
        t1.require(FDLayoutEdge, 'FDLayoutEdge');
        t1.require(FDLayoutNode, 'FDLayoutNode');
        t1.require(CoSEConstants, 'CoSEConstants');
        t1.require(CoSEEdge, 'CoSEEdge');
        t1.require(CoSEGraph, 'CoSEGraph');
        t1.require(CoSEGraphManager, 'CoSEGraphManager');
        t1.require(CoSELayout, 'CoSELayout');
        t1.require(CoSENode, 'CoSENode');
        t1.require(Compaction, 'Compaction');
        t1.require(SbgnPDConstants, 'SbgnPDConstants');
        t1.require(SbgnPDEdge, 'SbgnPDEdge');
        t1.require(SbgnPDLayout, 'SbgnPDLayout');
        t1.require(SbgnPDNode, 'SbgnPDNode');
        t1.require(SbgnProcessNode, 'SbgnProcessNode');
        t1.require(VisibilityEdge, 'VisibilityEdge');
        t1.require(VisibilityGraph, 'VisibilityGraph');
        t1.require(MemberPack, 'MemberPack');
        t1.require(Organization, 'Organization');
        t1.require(PolyominoQuickSort, 'PolyominoQuickSort');
        t1.require(PolyominoPacking, 'PolyominoPacking');
        t1.require(RectProc, 'RectProc');
    }

    var nodes = this.options.eles.nodes();
    var edges = this.options.eles.edges();

    // First I need to create the data structure to pass to the worker
    var pData = {
        'nodes': [],
        'edges': []
    };

    //Map the ids of nodes in the list to check if a node is in the list in constant time
    var nodeIdMap = {};

    //Fill the map in linear time
    for (var i = 0; i < nodes.length; i++) {
        nodeIdMap[nodes[i].id()] = true;
    }

    var lnodes = gm.getAllNodes();
    for (var i = 0; i < lnodes.length; i++) {
        var lnode = lnodes[i];
        var nodeId = lnode.id;
        var cyNode = this.options.cy.getElementById(nodeId);

        var parentId = cyNode.data('parent');
        parentId = nodeIdMap[parentId] ? parentId : undefined;

        var w = lnode.rect.width;
        var posX = lnode.rect.x;
        var posY = lnode.rect.y;
        var h = lnode.rect.height;
        var dummy_parent_id = null;
        
        // TODO: Is it correct?
        if (cyNode.scratch('sbgnPdLayout') && cyNode.scratch('sbgnPdLayout').dummy_parent_id)
            dummy_parent_id = cyNode.scratch('sbgnPdLayout').dummy_parent_id;

        pData[ 'nodes' ].push({
            id: nodeId,
            pid: parentId,
            x: posX,
            y: posY,
            width: w,
            height: h,
            dummy_parent_id: dummy_parent_id
        });

    }

    var ledges = gm.getAllEdges();
    for (var i = 0; i < ledges.length; i++) {
        var ledge = ledges[i];
        var edgeId = ledge.id;
        var cyEdge = this.options.cy.getElementById(edgeId);
        var srcNodeId = cyEdge.source().id();
        var tgtNodeId = cyEdge.target().id();
        pData[ 'edges' ].push({
            id: edgeId,
            source: srcNodeId,
            target: tgtNodeId
        });
    }

    var ready = false;

    t1.pass(pData).run(function (pData) {
        var log = function (msg) {
            broadcast({log: msg});
        };

        log("start thread");

        //the layout will be run in the thread and the results are to be passed
        //to the main thread with the result map
        var layout_t = new SbgnPDLayout();
        var gm_t = layout_t.newGraphManager();
        var ngraph = gm_t.layout.newGraph();
        var nnode = gm_t.layout.newNode(null);
        var root = gm_t.add(ngraph, nnode);
        root.graphManager = gm_t;
        gm_t.setRootGraph(root);
        var root_t = gm_t.rootGraph;

        //maps for inner usage of the thread
        var orphans_t = [];
        var idToLNode_t = {};
        var childrenMap = {};

        //A map of node id to corresponding node position and sizes
        //it is to be returned at the end of the thread function
        var result = {};

        //this function is similar to processChildrenList function in the main thread
        //it is to process the nodes in correct order recursively
        var processNodes = function (parent, children) {
            var size = children.length;
            for (var i = 0; i < size; i++) {
                var theChild = children[i];
                var children_of_children = childrenMap[theChild.id];
                var theNode;

                if (theChild.width != null
                        && theChild.height != null) {
                    theNode = parent.add(new CoSENode(gm_t,
                            new PointD(theChild.x, theChild.y),
                            new DimensionD(parseFloat(theChild.width),
                                    parseFloat(theChild.height))));
                } else {
                    theNode = parent.add(new CoSENode(gm_t));
                }
                theNode.id = theChild.id;
                idToLNode_t[theChild.id] = theNode;

                if (isNaN(theNode.rect.x)) {
                    theNode.rect.x = 0;
                }

                if (isNaN(theNode.rect.y)) {
                    theNode.rect.y = 0;
                }

                if (children_of_children != null && children_of_children.length > 0) {
                    var theNewGraph;
                    theNewGraph = layout_t.getGraphManager().add(layout_t.newGraph(), theNode);
                    theNewGraph.graphManager = gm_t;
                    processNodes(theNewGraph, children_of_children);
                }
            }
        }

        //fill the chidrenMap and orphans_t maps to process the nodes in the correct order
        var nodes = pData.nodes;
        for (var i = 0; i < nodes.length; i++) {
            var theNode = nodes[i];
            var p_id = theNode.pid;
            if (p_id != null) {
                if (childrenMap[p_id] == null) {
                    childrenMap[p_id] = [];
                }
                childrenMap[p_id].push(theNode);
            } else {
                orphans_t.push(theNode);
            }
        }

        processNodes(root_t, orphans_t);

        //handle the edges
        var edges = pData.edges;
        for (var i = 0; i < edges.length; i++) {
            var edge = edges[i];
            var sourceNode = idToLNode_t[edge.source];
            var targetNode = idToLNode_t[edge.target];
            var e1 = gm_t.add(layout_t.newEdge(), sourceNode, targetNode);
        }

        //run the layout crated in this thread
        layout_t.runLayout();

        //fill the result map
        for (var id in idToLNode_t) {
            var lNode = idToLNode_t[id];
            var rect = lNode.rect;
            result[id] = {
                id: id,
                x: rect.x,
                y: rect.y,
                w: rect.width,
                h: rect.height
            };
        }
        var seeds = {};
        seeds.rsSeed = RandomSeed.seed;
        seeds.rsX = RandomSeed.x;
        var pass = {
            result: result,
            seeds: seeds
        }
        //return the result map to pass it to the then function as parameter
        return pass;
    }).then(function (pass) {
        var result = pass.result;
        var seeds = pass.seeds;
        RandomSeed.seed = seeds.rsSeed;
        RandomSeed.x = seeds.rsX;
        //refresh the lnode positions and sizes by using result map
        for (var id in result) {
            var lNode = _SbgnPDLayout.idToLNode[id];
            var node = result[id];
            lNode.rect.x = node.x;
            lNode.rect.y = node.y;
            lNode.rect.width = node.w;
            lNode.rect.height = node.h;
        }
        if (after.options.tile) {
            // Repopulate members
            after.repopulateZeroDegreeMembers(tiledZeroDegreeNodes);
            after.repopulateCompounds(tiledMemberPack);
            after.options.eles.nodes().updateCompoundBounds();
        }

        var getPositions = function (i, ele) {
            var theId = ele.data('id');
            var lNode = _SbgnPDLayout.idToLNode[theId];

            return {
                x: lNode.getRect().getCenterX(),
                y: lNode.getRect().getCenterY()
            };
        };

        if (after.options.animate !== 'during') {
            after.options.eles.nodes().layoutPositions(after, after.options, getPositions);
        } else {
            after.options.eles.nodes().positions(getPositions);

            if (after.options.fit)
                after.options.cy.fit(after.options.eles.nodes(), after.options.padding);

            //trigger layoutready when each node has had its position set at least once
            if (!ready) {
                after.cy.one('layoutready', after.options.ready);
                after.cy.trigger('layoutready');
            }

            // trigger layoutstop when the layout stops (e.g. finishes)
            after.cy.one('layoutstop', after.options.stop);
            after.cy.trigger('layoutstop');
        }

        t1.stop();
        after.options.eles.nodes().removeScratch('sbgnPdLayout');
    });

    t1.on('message', function (e) {
        var logMsg = e.message.log;
        if (logMsg != null) {
            console.log('Thread log: ' + logMsg);
            return;
        }
        var pData = e.message.pData;
        if (pData != null) {
            after.options.eles.nodes().positions(function (i, ele) {
                if (ele.scratch('sbgnPdLayout') && ele.scratch('sbgnPdLayout').dummy_parent_id) {
                    var dummyParent = ele.scratch('sbgnPdLayout').dummy_parent_id;
                    return {
                        x: dummyParent.x,
                        y: dummyParent.y
                    };
                }
                var theId = ele.data('id');
                var pNode = pData[theId];
                var temp = this;
                while (pNode == null) {
                    temp = temp.parent()[0];
                    pNode = pData[temp.id()];
                    pData[theId] = pNode;
                }
                return {
                    x: pNode.x,
                    y: pNode.y
                };
            });

            if (after.options.fit)
                after.options.cy.fit(after.options.eles.nodes(), after.options.padding);

            if (!ready) {
                ready = true;
                after.one('layoutready', after.options.ready);
                after.trigger({type: 'layoutready', layout: after});
            }
            return;
        }
    });

    return this; // chaining
};

//Get the top most ones of a list of nodes
_SbgnPDLayout.getTopMostNodes = function (nodes) {
    var nodesMap = {};
    for (var i = 0; i < nodes.length; i++) {
        nodesMap[nodes[i].id()] = true;
    }
    var roots = nodes.filter(function (i, ele) {
        var parent = ele.parent()[0];
        while (parent != null) {
            if (nodesMap[parent.id()]) {
                return false;
            }
            parent = parent.parent()[0];
        }
        return true;
    });

    return roots;
};

//_SbgnPDLayout.prototype.getToBeTiled = function (node) {
//    var id = node.data("id");
//    //firstly check the previous results
//    if (_SbgnPDLayout.toBeTiled[id] != null) {
//        return _SbgnPDLayout.toBeTiled[id];
//    }
//
//    //only compound nodes are to be tiled
//    var children = node.children();
//    if (children == null || children.length == 0) {
//        _SbgnPDLayout.toBeTiled[id] = false;
//        return false;
//    }
//
//    //a compound node is not to be tiled if all of its compound children are not to be tiled
//    for (var i = 0; i < children.length; i++) {
//        var theChild = children[i];
//
//        if (this.getNodeDegree(theChild) > 0) {
//            _SbgnPDLayout.toBeTiled[id] = false;
//            return false;
//        }
//
//        //pass the children not having the compound structure
//        if (theChild.children() == null || theChild.children().length == 0) {
//            _SbgnPDLayout.toBeTiled[theChild.data("id")] = false;
//            continue;
//        }
//
//        if (!this.getToBeTiled(theChild)) {
//            _SbgnPDLayout.toBeTiled[id] = false;
//            return false;
//        }
//    }
//    _SbgnPDLayout.toBeTiled[id] = true;
//    return true;
//};

//_SbgnPDLayout.prototype.getNodeDegree = function (node) {
//    var id = node.id();
//    var edges = this.options.eles.edges().filter(function (i, ele) {
//        var source = ele.data('source');
//        var target = ele.data('target');
//        if (source != target && (source == id || target == id)) {
//            return true;
//        }
//    });
//    return edges.length;
//};

//_SbgnPDLayout.prototype.getNodeDegreeWithChildren = function (node) {
//    var degree = this.getNodeDegree(node);
//    var children = node.children();
//    for (var i = 0; i < children.length; i++) {
//        var child = children[i];
//        degree += this.getNodeDegreeWithChildren(child);
//    }
//    return degree;
//};

//_SbgnPDLayout.prototype.groupZeroDegreeMembers = function () {
//    // array of [parent_id x oneDegreeNode_id] 
//    var tempMemberGroups = [];
//    var memberGroups = [];
//    var self = this;
//    var parentMap = {};
//
//    for (var i = 0; i < this.options.eles.nodes().length; i++) {
//        parentMap[this.options.eles.nodes()[i].id()] = true;
//    }
//
//    // Find all zero degree nodes which aren't covered by a compound
//    var zeroDegree = this.options.eles.nodes().filter(function (i, ele) {
//        var pid = ele.data('parent');
//        if (pid != undefined && !parentMap[pid]) {
//            pid = undefined;
//        }
//
//        if (self.getNodeDegreeWithChildren(ele) == 0 && (pid == undefined || (pid != undefined && !self.getToBeTiled(ele.parent()[0]))))
//            return true;
//        else
//            return false;
//    });
//
//    // Create a map of parent node and its zero degree members
//    for (var i = 0; i < zeroDegree.length; i++)
//    {
//        var node = zeroDegree[i];
//        var p_id = node.parent().id();
//
//        if (p_id != undefined && !parentMap[p_id]) {
//            p_id = undefined;
//        }
//
//        if (typeof tempMemberGroups[p_id] === "undefined")
//            tempMemberGroups[p_id] = [];
//
//        tempMemberGroups[p_id] = tempMemberGroups[p_id].concat(node);
//    }
//
//    // If there are at least two nodes at a level, create a dummy compound for them
//    for (var p_id in tempMemberGroups) {
//        if (tempMemberGroups[p_id].length > 1) {
//            var dummyCompoundId = "DummyCompound_" + p_id;
//            memberGroups[dummyCompoundId] = tempMemberGroups[p_id];
//
//            // Create a dummy compound
//            if (this.options.cy.getElementById(dummyCompoundId).empty()) {
//                this.options.cy.add({
//                    group: "nodes",
//                    data: {id: dummyCompoundId, parent: p_id
//                    }
//                });
//
//                var dummy = this.options.cy.nodes()[this.options.cy.nodes().length - 1];
//                this.options.eles = this.options.eles.union(dummy);
//                dummy.hide();
//
//                for (var i = 0; i < tempMemberGroups[p_id].length; i++) {
//                    if (i == 0) {
//                        dummy.scratch('sbgnPdLayout', {tempchildren: []});
//                    }
//                    var node = tempMemberGroups[p_id][i];
//                    var scratchObj = node.scratch('sbgnPdLayout');
//                    if (!scratchObj) {
//                        scratchObj = {};
//                        node.scratch('sbgnPdLayout', scratchObj);
//                    }
//                    scratchObj['dummy_parent_id'] = dummyCompoundId;
//                    this.options.cy.add({
//                        group: "nodes",
//                        data: {parent: dummyCompoundId, width: node.width(), height: node.height()
//                        }
//                    });
//                    var tempchild = this.options.cy.nodes()[this.options.cy.nodes().length - 1];
//                    tempchild.hide();
//                    tempchild.css('width', tempchild.data('width'));
//                    tempchild.css('height', tempchild.data('height'));
//                    tempchild.width();
//                    dummy.scratch('sbgnPdLayout').tempchildren.push(tempchild);
//                }
//            }
//        }
//    }
//
//    return memberGroups;
//};

//_SbgnPDLayout.prototype.performDFSOnCompounds = function (options) {
//    var compoundOrder = [];
//
//    var roots = _SbgnPDLayout.getTopMostNodes(this.options.eles.nodes());
//    this.fillCompexOrderByDFS(compoundOrder, roots);
//
//    return compoundOrder;
//};

//_SbgnPDLayout.prototype.fillCompexOrderByDFS = function (compoundOrder, children) {
//    for (var i = 0; i < children.length; i++) {
//        var child = children[i];
//        this.fillCompexOrderByDFS(compoundOrder, child.children());
//        if (this.getToBeTiled(child)) {
//            compoundOrder.push(child);
//        }
//    }
//};

//_SbgnPDLayout.prototype.clearCompounds = function (options) {
//    var childGraphMap = [];
//
//    // Get compound ordering by finding the inner one first
//    var compoundOrder = this.performDFSOnCompounds(options);
//    _SbgnPDLayout.compoundOrder = compoundOrder;
//    this.processChildrenList(this.root, _SbgnPDLayout.getTopMostNodes(this.options.eles.nodes()));
//
//    for (var i = 0; i < compoundOrder.length; i++) {
//        // find the corresponding layout node
//        var lCompoundNode = _SbgnPDLayout.idToLNode[compoundOrder[i].id()];
//
//        childGraphMap[compoundOrder[i].id()] = compoundOrder[i].children();
//
//        // Remove children of compounds 
//        lCompoundNode.child = null;
//    }
//
//    // Tile the removed children
//    var tiledMemberPack = this.tileCompoundMembers(childGraphMap);
//
//    return tiledMemberPack;
//};

//_SbgnPDLayout.prototype.clearZeroDegreeMembers = function (memberGroups) {
//    var tiledZeroDegreePack = [];
//
//    for (var id in memberGroups) {
//        var compoundNode = _SbgnPDLayout.idToLNode[id];
//
//        tiledZeroDegreePack[id] = this.tileNodes(memberGroups[id]);
//
//        // Set the width and height of the dummy compound as calculated
//        compoundNode.rect.width = tiledZeroDegreePack[id].width;
//        compoundNode.rect.height = tiledZeroDegreePack[id].height;
//    }
//    return tiledZeroDegreePack;
//};

//_SbgnPDLayout.prototype.repopulateCompounds = function (tiledMemberPack) {
//    for (var i = _SbgnPDLayout.compoundOrder.length - 1; i >= 0; i--) {
//        var id = _SbgnPDLayout.compoundOrder[i].id();
//        var lCompoundNode = _SbgnPDLayout.idToLNode[id];
//        var horizontalMargin = parseInt(_SbgnPDLayout.compoundOrder[i].css('padding-left'));
//        var verticalMargin = parseInt(_SbgnPDLayout.compoundOrder[i].css('padding-top'));
//
//        this.adjustLocations(tiledMemberPack[id], lCompoundNode.rect.x, lCompoundNode.rect.y, horizontalMargin, verticalMargin);
//    }
//};
//
//_SbgnPDLayout.prototype.repopulateZeroDegreeMembers = function (tiledPack) {
//    for (var i in tiledPack) {
//        var compound = this.cy.getElementById(i);
//        var compoundNode = _SbgnPDLayout.idToLNode[i];
//        var horizontalMargin = parseInt(compound.css('padding-left'));
//        var verticalMargin = parseInt(compound.css('padding-top'));
//
//        // Adjust the positions of nodes wrt its compound
//        this.adjustLocations(tiledPack[i], compoundNode.rect.x, compoundNode.rect.y, horizontalMargin, verticalMargin);
//
//        var tempchildren = compound.scratch('sbgnPdLayout').tempchildren;
//        for (var i = 0; i < tempchildren.length; i++) {
//            tempchildren[i].remove();
//        }
//
//        // Remove the dummy compound
//        compound.remove();
//    }
//};

/**
 * This method places each zero degree member wrt given (x,y) coordinates (top left). 
 */
//_SbgnPDLayout.prototype.adjustLocations = function (organization, x, y, compoundHorizontalMargin, compoundVerticalMargin) {
//    x += compoundHorizontalMargin;
//    y += compoundVerticalMargin;
//
//    var left = x;
//
//    for (var i = 0; i < organization.rows.length; i++) {
//        var row = organization.rows[i];
//        x = left;
//        var maxHeight = 0;
//
//        for (var j = 0; j < row.length; j++) {
//            var lnode = row[j];
//            var node = this.cy.getElementById(lnode.id);
//
//            lnode.rect.x = x;// + lnode.rect.width / 2;
//            lnode.rect.y = y;// + lnode.rect.height / 2;
//
//            x += lnode.rect.width + organization.horizontalPadding;
//
//            if (lnode.rect.height > maxHeight)
//                maxHeight = lnode.rect.height;
//        }
//
//        y += maxHeight + organization.verticalPadding;
//    }
//};

//_SbgnPDLayout.prototype.tileCompoundMembers = function (childGraphMap) {
//    var tiledMemberPack = [];
//
//    for (var id in childGraphMap) {
//        // Access layoutInfo nodes to set the width and height of compounds
//        var compoundNode = _SbgnPDLayout.idToLNode[id];
//
//        tiledMemberPack[id] = this.tileNodes(childGraphMap[id]);
//
//        compoundNode.rect.width = tiledMemberPack[id].width + 20;
//        compoundNode.rect.height = tiledMemberPack[id].height + 20;
//    }
//
//    return tiledMemberPack;
//};

//_SbgnPDLayout.prototype.tileNodes = function (nodes) {
//    var self = this;
//    var verticalPadding = typeof self.options.tilingPaddingVertical === 'function' ? self.options.tilingPaddingVertical.call() : self.options.tilingPaddingVertical;
//    var horizontalPadding = typeof self.options.tilingPaddingHorizontal === 'function' ? self.options.tilingPaddingHorizontal.call() : self.options.tilingPaddingHorizontal;
//    var organization = {
//        rows: [],
//        rowWidth: [],
//        rowHeight: [],
//        width: 20,
//        height: 20,
//        verticalPadding: verticalPadding,
//        horizontalPadding: horizontalPadding
//    };
//
//    var layoutNodes = [];
//
//    // Get layout nodes
//    for (var i = 0; i < nodes.length; i++) {
//        var node = nodes[i];
//        var lNode = _SbgnPDLayout.idToLNode[node.id()];
//
//        if (!node.scratch('coseBilkent') || !node.scratch('coseBilkent').dummy_parent_id) {
//            var owner = lNode.owner;
//            owner.remove(lNode);
//
//            this.gm.resetAllNodes();
//            this.gm.getAllNodes();
//        }
//
//        layoutNodes.push(lNode);
//    }
//
//    // Sort the nodes in ascending order of their areas
//    layoutNodes.sort(function (n1, n2) {
//        if (n1.rect.width * n1.rect.height > n2.rect.width * n2.rect.height)
//            return -1;
//        if (n1.rect.width * n1.rect.height < n2.rect.width * n2.rect.height)
//            return 1;
//        return 0;
//    });
//
//    // Create the organization -> tile members
//    for (var i = 0; i < layoutNodes.length; i++) {
//        var lNode = layoutNodes[i];
//
//        var cyNode = this.cy.getElementById(lNode.id).parent()[0];
//        var minWidth = 0;
//        if (cyNode) {
//            minWidth = parseInt(cyNode.css('padding-left')) + parseInt(cyNode.css('padding-right'));
//        }
//
//        if (organization.rows.length == 0) {
//            this.insertNodeToRow(organization, lNode, 0, minWidth);
//        } else if (this.canAddHorizontal(organization, lNode.rect.width, lNode.rect.height)) {
//            this.insertNodeToRow(organization, lNode, this.getShortestRowIndex(organization), minWidth);
//        } else {
//            this.insertNodeToRow(organization, lNode, organization.rows.length, minWidth);
//        }
//
//        this.shiftToLastRow(organization);
//    }
//
//    return organization;
//};
//
//_SbgnPDLayout.prototype.insertNodeToRow = function (organization, node, rowIndex, minWidth) {
//    var minCompoundSize = minWidth;
//
//    // Add new row if needed
//    if (rowIndex == organization.rows.length) {
//        var secondDimension = [];
//
//        organization.rows.push(secondDimension);
//        organization.rowWidth.push(minCompoundSize);
//        organization.rowHeight.push(0);
//    }
//
//    // Update row width
//    var w = organization.rowWidth[rowIndex] + node.rect.width;
//
//    if (organization.rows[rowIndex].length > 0) {
//        w += organization.horizontalPadding;
//    }
//
//    organization.rowWidth[rowIndex] = w;
//    // Update compound width
//    if (organization.width < w) {
//        organization.width = w;
//    }
//
//    // Update height
//    var h = node.rect.height;
//    if (rowIndex > 0)
//        h += organization.verticalPadding;
//
//    var extraHeight = 0;
//    if (h > organization.rowHeight[rowIndex]) {
//        extraHeight = organization.rowHeight[rowIndex];
//        organization.rowHeight[rowIndex] = h;
//        extraHeight = organization.rowHeight[rowIndex] - extraHeight;
//    }
//
//    organization.height += extraHeight;
//
//    // Insert node
//    organization.rows[rowIndex].push(node);
//};
//
////Scans the rows of an organization and returns the one with the min width
//_SbgnPDLayout.prototype.getShortestRowIndex = function (organization) {
//    var r = -1;
//    var min = Number.MAX_VALUE;
//
//    for (var i = 0; i < organization.rows.length; i++) {
//        if (organization.rowWidth[i] < min) {
//            r = i;
//            min = organization.rowWidth[i];
//        }
//    }
//    return r;
//};
//
////Scans the rows of an organization and returns the one with the max width
//_SbgnPDLayout.prototype.getLongestRowIndex = function (organization) {
//    var r = -1;
//    var max = Number.MIN_VALUE;
//
//    for (var i = 0; i < organization.rows.length; i++) {
//
//        if (organization.rowWidth[i] > max) {
//            r = i;
//            max = organization.rowWidth[i];
//        }
//    }
//
//    return r;
//};
//
///**
// * This method checks whether adding extra width to the organization violates
// * the aspect ratio(1) or not.
// */
//_SbgnPDLayout.prototype.canAddHorizontal = function (organization, extraWidth, extraHeight) {
//
//    var sri = this.getShortestRowIndex(organization);
//
//    if (sri < 0) {
//        return true;
//    }
//
//    var min = organization.rowWidth[sri];
//
//    if (min + organization.horizontalPadding + extraWidth <= organization.width)
//        return true;
//
//    var hDiff = 0;
//
//    // Adding to an existing row
//    if (organization.rowHeight[sri] < extraHeight) {
//        if (sri > 0)
//            hDiff = extraHeight + organization.verticalPadding - organization.rowHeight[sri];
//    }
//
//    var add_to_row_ratio;
//    if (organization.width - min >= extraWidth + organization.horizontalPadding) {
//        add_to_row_ratio = (organization.height + hDiff) / (min + extraWidth + organization.horizontalPadding);
//    } else {
//        add_to_row_ratio = (organization.height + hDiff) / organization.width;
//    }
//
//    // Adding a new row for this node
//    hDiff = extraHeight + organization.verticalPadding;
//    var add_new_row_ratio;
//    if (organization.width < extraWidth) {
//        add_new_row_ratio = (organization.height + hDiff) / extraWidth;
//    } else {
//        add_new_row_ratio = (organization.height + hDiff) / organization.width;
//    }
//
//    if (add_new_row_ratio < 1)
//        add_new_row_ratio = 1 / add_new_row_ratio;
//
//    if (add_to_row_ratio < 1)
//        add_to_row_ratio = 1 / add_to_row_ratio;
//
//    return add_to_row_ratio < add_new_row_ratio;
//};
//
//
////If moving the last node from the longest row and adding it to the last
////row makes the bounding box smaller, do it.
//_SbgnPDLayout.prototype.shiftToLastRow = function (organization) {
//    var longest = this.getLongestRowIndex(organization);
//    var last = organization.rowWidth.length - 1;
//    var row = organization.rows[longest];
//    var node = row[row.length - 1];
//
//    var diff = node.width + organization.horizontalPadding;
//
//    // Check if there is enough space on the last row
//    if (organization.width - organization.rowWidth[last] > diff && longest != last) {
//        // Remove the last element of the longest row
//        row.splice(-1, 1);
//
//        // Push it to the last row
//        organization.rows[last].push(node);
//
//        organization.rowWidth[longest] = organization.rowWidth[longest] - diff;
//        organization.rowWidth[last] = organization.rowWidth[last] + diff;
//        organization.width = organization.rowWidth[this.getLongestRowIndex(organization)];
//
//        // Update heights of the organization
//        var maxHeight = Number.MIN_VALUE;
//        for (var i = 0; i < row.length; i++) {
//            if (row[i].height > maxHeight)
//                maxHeight = row[i].height;
//        }
//        if (longest > 0)
//            maxHeight += organization.verticalPadding;
//
//        var prevTotal = organization.rowHeight[longest] + organization.rowHeight[last];
//
//        organization.rowHeight[longest] = maxHeight;
//        if (organization.rowHeight[last] < node.height + organization.verticalPadding)
//            organization.rowHeight[last] = node.height + organization.verticalPadding;
//
//        var finalTotal = organization.rowHeight[longest] + organization.rowHeight[last];
//        organization.height += (finalTotal - prevTotal);
//
//        this.shiftToLastRow(organization);
//    }
//};

/**
 * @brief : called on continuous layouts to stop them before they finish
 */
_SbgnPDLayout.prototype.stop = function () {
    this.stopped = true;

    if (this.thread) {
        this.thread.stop();
    }

    this.trigger('layoutstop');

    return this; // chaining
};

_SbgnPDLayout.prototype.processChildrenList = function (parent, children) {
    var size = children.length;
    for (var i = 0; i < size; i++) {
        var theChild = children[i];
        this.options.eles.nodes().length;
        var children_of_children = theChild.children();
        var theNode;

        if (theChild.width() != null
                && theChild.height() != null) {
            theNode = parent.add(new CoSENode(_SbgnPDLayout.layout.graphManager,
                    new PointD(theChild.position('x'), theChild.position('y')),
                    new DimensionD(parseFloat(theChild.width()),
                            parseFloat(theChild.height()))));
        } else {
            theNode = parent.add(new CoSENode(this.graphManager));
        }
        theNode.id = theChild.data("id");
        _SbgnPDLayout.idToLNode[theChild.data("id")] = theNode;

        if (isNaN(theNode.rect.x)) {
            theNode.rect.x = 0;
        }

        if (isNaN(theNode.rect.y)) {
            theNode.rect.y = 0;
        }

        if (children_of_children != null && children_of_children.length > 0) {
            var theNewGraph;
            theNewGraph = _SbgnPDLayout.layout.getGraphManager().add(_SbgnPDLayout.layout.newGraph(), theNode);
            this.processChildrenList(theNewGraph, children_of_children);
        }
    }
};

module.exports = function get(cytoscape) {
    Thread = cytoscape.Thread;

    return _SbgnPDLayout;
};

},{"./CoSEConstants":1,"./CoSEEdge":2,"./CoSEGraph":3,"./CoSEGraphManager":4,"./CoSELayout":5,"./CoSENode":6,"./Compaction":7,"./DimensionD":8,"./FDLayout":9,"./FDLayoutConstants":10,"./FDLayoutEdge":11,"./FDLayoutNode":12,"./HashMap":13,"./HashSet":14,"./IGeometry":15,"./IMath":16,"./Integer":17,"./LEdge":18,"./LGraph":19,"./LGraphManager":20,"./LGraphObject":21,"./LNode":22,"./Layout":23,"./LayoutConstants":24,"./MemberPack":25,"./Organization":26,"./Point":27,"./PointD":28,"./PolyominoPacking":30,"./PolyominoQuickSort":31,"./RandomSeed":32,"./RectProc":33,"./RectangleD":34,"./SbgnPDConstants":35,"./SbgnPDEdge":36,"./SbgnPDLayout":37,"./SbgnPDNode":38,"./SbgnProcessNode":39,"./Transform":40,"./UniqueIDGeneretor":41,"./VisibilityEdge":42,"./VisibilityGraph":43}],45:[function(_dereq_,module,exports){
'use strict';

// registers the extension on a cytoscape lib ref
var getLayout = _dereq_('./Layout');
//var getUtilities = require('./Utilities');

var register = function( cytoscape ){
  var Layout = getLayout( cytoscape );
  //var Utilities = getUtilities ( cytoscape );
  
  cytoscape('layout', 'sbgnPdLayout', Layout);
  //cytoscape('core', 'utilities', Utilities);
};

if( typeof cytoscape !== 'undefined' ){ // expose to global cytoscape (i.e. window.cytoscape)
  register( cytoscape );
}

module.exports = register;

},{"./Layout":44}]},{},[45])(45)
});
//# sourceMappingURL=data:application/json;charset:utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyaWZ5L25vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJzcmMvTGF5b3V0L0NvU0VDb25zdGFudHMuanMiLCJzcmMvTGF5b3V0L0NvU0VFZGdlLmpzIiwic3JjL0xheW91dC9Db1NFR3JhcGguanMiLCJzcmMvTGF5b3V0L0NvU0VHcmFwaE1hbmFnZXIuanMiLCJzcmMvTGF5b3V0L0NvU0VMYXlvdXQuanMiLCJzcmMvTGF5b3V0L0NvU0VOb2RlLmpzIiwic3JjL0xheW91dC9Db21wYWN0aW9uLmpzIiwic3JjL0xheW91dC9EaW1lbnNpb25ELmpzIiwic3JjL0xheW91dC9GRExheW91dC5qcyIsInNyYy9MYXlvdXQvRkRMYXlvdXRDb25zdGFudHMuanMiLCJzcmMvTGF5b3V0L0ZETGF5b3V0RWRnZS5qcyIsInNyYy9MYXlvdXQvRkRMYXlvdXROb2RlLmpzIiwic3JjL0xheW91dC9IYXNoTWFwLmpzIiwic3JjL0xheW91dC9IYXNoU2V0LmpzIiwic3JjL0xheW91dC9JR2VvbWV0cnkuanMiLCJzcmMvTGF5b3V0L0lNYXRoLmpzIiwic3JjL0xheW91dC9JbnRlZ2VyLmpzIiwic3JjL0xheW91dC9MRWRnZS5qcyIsInNyYy9MYXlvdXQvTEdyYXBoLmpzIiwic3JjL0xheW91dC9MR3JhcGhNYW5hZ2VyLmpzIiwic3JjL0xheW91dC9MR3JhcGhPYmplY3QuanMiLCJzcmMvTGF5b3V0L0xOb2RlLmpzIiwic3JjL0xheW91dC9MYXlvdXQuanMiLCJzcmMvTGF5b3V0L0xheW91dENvbnN0YW50cy5qcyIsInNyYy9MYXlvdXQvTWVtYmVyUGFjay5qcyIsInNyYy9MYXlvdXQvT3JnYW5pemF0aW9uLmpzIiwic3JjL0xheW91dC9Qb2ludC5qcyIsInNyYy9MYXlvdXQvUG9pbnRELmpzIiwic3JjL0xheW91dC9Qb2x5b21pbm8uanMiLCJzcmMvTGF5b3V0L1BvbHlvbWlub1BhY2tpbmcuanMiLCJzcmMvTGF5b3V0L1BvbHlvbWlub1F1aWNrU29ydC5qcyIsInNyYy9MYXlvdXQvUmFuZG9tU2VlZC5qcyIsInNyYy9MYXlvdXQvUmVjdFByb2MuanMiLCJzcmMvTGF5b3V0L1JlY3RhbmdsZUQuanMiLCJzcmMvTGF5b3V0L1NiZ25QRENvbnN0YW50cy5qcyIsInNyYy9MYXlvdXQvU2JnblBERWRnZS5qcyIsInNyYy9MYXlvdXQvU2JnblBETGF5b3V0LmpzIiwic3JjL0xheW91dC9TYmduUEROb2RlLmpzIiwic3JjL0xheW91dC9TYmduUHJvY2Vzc05vZGUuanMiLCJzcmMvTGF5b3V0L1RyYW5zZm9ybS5qcyIsInNyYy9MYXlvdXQvVW5pcXVlSURHZW5lcmV0b3IuanMiLCJzcmMvTGF5b3V0L1Zpc2liaWxpdHlFZGdlLmpzIiwic3JjL0xheW91dC9WaXNpYmlsaXR5R3JhcGguanMiLCJzcmMvTGF5b3V0L2luZGV4LmpzIiwic3JjL2luZGV4LmpzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0FDQUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDZkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDWkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDWkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDWkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3phQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDdkhBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDN01BO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzlCQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5V0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDOUJBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2REE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzFaQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNQQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3ZKQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ25jQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN0ZUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ0xBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDN1ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN0cEJBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDbEZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDeEZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDaFFBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDekVBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2hEQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNmQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDalFBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN6YUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ1hBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5SkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNsSUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN0RUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDM0VBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDemhEQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3ZNQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2ovQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzNKQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDN0JBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3BDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMzVEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQy9rQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQSIsImZpbGUiOiJnZW5lcmF0ZWQuanMiLCJzb3VyY2VSb290IjoiIiwic291cmNlc0NvbnRlbnQiOlsiKGZ1bmN0aW9uIGUodCxuLHIpe2Z1bmN0aW9uIHMobyx1KXtpZighbltvXSl7aWYoIXRbb10pe3ZhciBhPXR5cGVvZiByZXF1aXJlPT1cImZ1bmN0aW9uXCImJnJlcXVpcmU7aWYoIXUmJmEpcmV0dXJuIGEobywhMCk7aWYoaSlyZXR1cm4gaShvLCEwKTt2YXIgZj1uZXcgRXJyb3IoXCJDYW5ub3QgZmluZCBtb2R1bGUgJ1wiK28rXCInXCIpO3Rocm93IGYuY29kZT1cIk1PRFVMRV9OT1RfRk9VTkRcIixmfXZhciBsPW5bb109e2V4cG9ydHM6e319O3Rbb11bMF0uY2FsbChsLmV4cG9ydHMsZnVuY3Rpb24oZSl7dmFyIG49dFtvXVsxXVtlXTtyZXR1cm4gcyhuP246ZSl9LGwsbC5leHBvcnRzLGUsdCxuLHIpfXJldHVybiBuW29dLmV4cG9ydHN9dmFyIGk9dHlwZW9mIHJlcXVpcmU9PVwiZnVuY3Rpb25cIiYmcmVxdWlyZTtmb3IodmFyIG89MDtvPHIubGVuZ3RoO28rKylzKHJbb10pO3JldHVybiBzfSkiLCJ2YXIgRkRMYXlvdXRDb25zdGFudHMgPSByZXF1aXJlKCcuL0ZETGF5b3V0Q29uc3RhbnRzJyk7XG5cbmZ1bmN0aW9uIENvU0VDb25zdGFudHMoKSB7XG59XG5cbi8vQ29TRUNvbnN0YW50cyBpbmhlcml0cyBzdGF0aWMgcHJvcHMgaW4gRkRMYXlvdXRDb25zdGFudHNcbmZvciAodmFyIHByb3AgaW4gRkRMYXlvdXRDb25zdGFudHMpIHtcbiAgQ29TRUNvbnN0YW50c1twcm9wXSA9IEZETGF5b3V0Q29uc3RhbnRzW3Byb3BdO1xufVxuXG5Db1NFQ29uc3RhbnRzLkRFRkFVTFRfVVNFX01VTFRJX0xFVkVMX1NDQUxJTkcgPSBmYWxzZTtcbkNvU0VDb25zdGFudHMuREVGQVVMVF9SQURJQUxfU0VQQVJBVElPTiA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEg7XG5Db1NFQ29uc3RhbnRzLkRFRkFVTFRfQ09NUE9ORU5UX1NFUEVSQVRJT04gPSA2MDtcblxubW9kdWxlLmV4cG9ydHMgPSBDb1NFQ29uc3RhbnRzO1xuIiwidmFyIEZETGF5b3V0RWRnZSA9IHJlcXVpcmUoJy4vRkRMYXlvdXRFZGdlJyk7XG5cbmZ1bmN0aW9uIENvU0VFZGdlKHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSkge1xuICBGRExheW91dEVkZ2UuY2FsbCh0aGlzLCBzb3VyY2UsIHRhcmdldCwgdkVkZ2UpO1xufVxuXG5Db1NFRWRnZS5wcm90b3R5cGUgPSBPYmplY3QuY3JlYXRlKEZETGF5b3V0RWRnZS5wcm90b3R5cGUpO1xuZm9yICh2YXIgcHJvcCBpbiBGRExheW91dEVkZ2UpIHtcbiAgQ29TRUVkZ2VbcHJvcF0gPSBGRExheW91dEVkZ2VbcHJvcF07XG59XG5cbm1vZHVsZS5leHBvcnRzID0gQ29TRUVkZ2U7XG4iLCJ2YXIgTEdyYXBoID0gcmVxdWlyZSgnLi9MR3JhcGgnKTtcblxuZnVuY3Rpb24gQ29TRUdyYXBoKHBhcmVudCwgZ3JhcGhNZ3IsIHZHcmFwaCkge1xuICBMR3JhcGguY2FsbCh0aGlzLCBwYXJlbnQsIGdyYXBoTWdyLCB2R3JhcGgpO1xufVxuXG5Db1NFR3JhcGgucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMR3JhcGgucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gTEdyYXBoKSB7XG4gIENvU0VHcmFwaFtwcm9wXSA9IExHcmFwaFtwcm9wXTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBDb1NFR3JhcGg7XG4iLCJ2YXIgTEdyYXBoTWFuYWdlciA9IHJlcXVpcmUoJy4vTEdyYXBoTWFuYWdlcicpO1xuXG5mdW5jdGlvbiBDb1NFR3JhcGhNYW5hZ2VyKGxheW91dCkge1xuICBMR3JhcGhNYW5hZ2VyLmNhbGwodGhpcywgbGF5b3V0KTtcbn1cblxuQ29TRUdyYXBoTWFuYWdlci5wcm90b3R5cGUgPSBPYmplY3QuY3JlYXRlKExHcmFwaE1hbmFnZXIucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gTEdyYXBoTWFuYWdlcikge1xuICBDb1NFR3JhcGhNYW5hZ2VyW3Byb3BdID0gTEdyYXBoTWFuYWdlcltwcm9wXTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBDb1NFR3JhcGhNYW5hZ2VyO1xuIiwidmFyIEZETGF5b3V0ID0gcmVxdWlyZSgnLi9GRExheW91dCcpO1xudmFyIENvU0VHcmFwaE1hbmFnZXIgPSByZXF1aXJlKCcuL0NvU0VHcmFwaE1hbmFnZXInKTtcbnZhciBDb1NFR3JhcGggPSByZXF1aXJlKCcuL0NvU0VHcmFwaCcpO1xudmFyIENvU0VOb2RlID0gcmVxdWlyZSgnLi9Db1NFTm9kZScpO1xudmFyIENvU0VFZGdlID0gcmVxdWlyZSgnLi9Db1NFRWRnZScpO1xuXG5mdW5jdGlvbiBDb1NFTGF5b3V0KCkge1xuICBGRExheW91dC5jYWxsKHRoaXMpO1xufVxuXG5Db1NFTGF5b3V0LnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoRkRMYXlvdXQucHJvdG90eXBlKTtcblxuZm9yICh2YXIgcHJvcCBpbiBGRExheW91dCkge1xuICBDb1NFTGF5b3V0W3Byb3BdID0gRkRMYXlvdXRbcHJvcF07XG59XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLm5ld0dyYXBoTWFuYWdlciA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGdtID0gbmV3IENvU0VHcmFwaE1hbmFnZXIodGhpcyk7XG4gIHRoaXMuZ3JhcGhNYW5hZ2VyID0gZ207XG4gIHJldHVybiBnbTtcbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLm5ld0dyYXBoID0gZnVuY3Rpb24gKHZHcmFwaCkge1xuICByZXR1cm4gbmV3IENvU0VHcmFwaChudWxsLCB0aGlzLmdyYXBoTWFuYWdlciwgdkdyYXBoKTtcbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLm5ld05vZGUgPSBmdW5jdGlvbiAodk5vZGUpIHtcbiAgcmV0dXJuIG5ldyBDb1NFTm9kZSh0aGlzLmdyYXBoTWFuYWdlciwgdk5vZGUpO1xufTtcblxuQ29TRUxheW91dC5wcm90b3R5cGUubmV3RWRnZSA9IGZ1bmN0aW9uICh2RWRnZSkge1xuICByZXR1cm4gbmV3IENvU0VFZGdlKG51bGwsIG51bGwsIHZFZGdlKTtcbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLmluaXRQYXJhbWV0ZXJzID0gZnVuY3Rpb24gKCkge1xuICBGRExheW91dC5wcm90b3R5cGUuaW5pdFBhcmFtZXRlcnMuY2FsbCh0aGlzLCBhcmd1bWVudHMpO1xuICBpZiAoIXRoaXMuaXNTdWJMYXlvdXQpIHtcbiAgICBpZiAoQ29TRUNvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIIDwgMTApXG4gICAge1xuICAgICAgdGhpcy5pZGVhbEVkZ2VMZW5ndGggPSAxMDtcbiAgICB9XG4gICAgZWxzZVxuICAgIHtcbiAgICAgIHRoaXMuaWRlYWxFZGdlTGVuZ3RoID0gQ29TRUNvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIO1xuICAgIH1cblxuICAgIHRoaXMudXNlU21hcnRJZGVhbEVkZ2VMZW5ndGhDYWxjdWxhdGlvbiA9XG4gICAgICAgICAgICBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfVVNFX1NNQVJUX0lERUFMX0VER0VfTEVOR1RIX0NBTENVTEFUSU9OO1xuICAgIHRoaXMuc3ByaW5nQ29uc3RhbnQgPVxuICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9TUFJJTkdfU1RSRU5HVEg7XG4gICAgdGhpcy5yZXB1bHNpb25Db25zdGFudCA9XG4gICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX1JFUFVMU0lPTl9TVFJFTkdUSDtcbiAgICB0aGlzLmdyYXZpdHlDb25zdGFudCA9XG4gICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0dSQVZJVFlfU1RSRU5HVEg7XG4gICAgdGhpcy5jb21wb3VuZEdyYXZpdHlDb25zdGFudCA9XG4gICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0NPTVBPVU5EX0dSQVZJVFlfU1RSRU5HVEg7XG4gICAgdGhpcy5ncmF2aXR5UmFuZ2VGYWN0b3IgPVxuICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUjtcbiAgICB0aGlzLmNvbXBvdW5kR3Jhdml0eVJhbmdlRmFjdG9yID1cbiAgICAgICAgICAgIEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9SQU5HRV9GQUNUT1I7XG4gIH1cbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLmxheW91dCA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGNyZWF0ZUJlbmRzQXNOZWVkZWQgPSBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9DUkVBVEVfQkVORFNfQVNfTkVFREVEO1xuICBpZiAoY3JlYXRlQmVuZHNBc05lZWRlZClcbiAge1xuICAgIHRoaXMuY3JlYXRlQmVuZHBvaW50cygpO1xuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsRWRnZXMoKTtcbiAgfVxuXG4gIHRoaXMubGV2ZWwgPSAwO1xuICByZXR1cm4gdGhpcy5jbGFzc2ljTGF5b3V0KCk7XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5jbGFzc2ljTGF5b3V0ID0gZnVuY3Rpb24gKCkge1xuICB0aGlzLmNhbGN1bGF0ZU5vZGVzVG9BcHBseUdyYXZpdGF0aW9uVG8oKTtcbiAgdGhpcy5ncmFwaE1hbmFnZXIuY2FsY0xvd2VzdENvbW1vbkFuY2VzdG9ycygpO1xuICB0aGlzLmdyYXBoTWFuYWdlci5jYWxjSW5jbHVzaW9uVHJlZURlcHRocygpO1xuICB0aGlzLmdyYXBoTWFuYWdlci5nZXRSb290KCkuY2FsY0VzdGltYXRlZFNpemUoKTtcbiAgdGhpcy5jYWxjSWRlYWxFZGdlTGVuZ3RocygpO1xuICBpZiAoIXRoaXMuaW5jcmVtZW50YWwpXG4gIHtcbiAgICB2YXIgZm9yZXN0ID0gdGhpcy5nZXRGbGF0Rm9yZXN0KCk7XG5cbiAgICAvLyBUaGUgZ3JhcGggYXNzb2NpYXRlZCB3aXRoIHRoaXMgbGF5b3V0IGlzIGZsYXQgYW5kIGEgZm9yZXN0XG4gICAgaWYgKGZvcmVzdC5sZW5ndGggPiAwKVxuXG4gICAge1xuICAgICAgdGhpcy5wb3NpdGlvbk5vZGVzUmFkaWFsbHkoZm9yZXN0KTtcbiAgICB9XG4gICAgLy8gVGhlIGdyYXBoIGFzc29jaWF0ZWQgd2l0aCB0aGlzIGxheW91dCBpcyBub3QgZmxhdCBvciBhIGZvcmVzdFxuICAgIGVsc2VcbiAgICB7XG4gICAgICB0aGlzLnBvc2l0aW9uTm9kZXNSYW5kb21seSgpO1xuICAgIH1cbiAgfVxuXG4gIHRoaXMuaW5pdFNwcmluZ0VtYmVkZGVyKCk7XG4gIHRoaXMucnVuU3ByaW5nRW1iZWRkZXIoKTtcblxuICBjb25zb2xlLmxvZyhcIkNsYXNzaWMgQ29TRSBsYXlvdXQgZmluaXNoZWQgYWZ0ZXIgXCIgK1xuICAgICAgICAgIHRoaXMudG90YWxJdGVyYXRpb25zICsgXCIgaXRlcmF0aW9uc1wiKTtcblxuICByZXR1cm4gdHJ1ZTtcbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLnJ1blNwcmluZ0VtYmVkZGVyID0gZnVuY3Rpb24gKCkge1xuICB2YXIgbGFzdEZyYW1lID0gbmV3IERhdGUoKS5nZXRUaW1lKCk7XG4gIHZhciBpbml0aWFsQW5pbWF0aW9uUGVyaW9kID0gMjU7XG4gIHZhciBhbmltYXRpb25QZXJpb2QgPSBpbml0aWFsQW5pbWF0aW9uUGVyaW9kO1xuICBkb1xuICB7XG4gICAgdGhpcy50b3RhbEl0ZXJhdGlvbnMrKztcblxuICAgIGlmICh0aGlzLnRvdGFsSXRlcmF0aW9ucyAlIEZETGF5b3V0Q29uc3RhbnRzLkNPTlZFUkdFTkNFX0NIRUNLX1BFUklPRCA9PSAwKVxuICAgIHtcbiAgICAgIGlmICh0aGlzLmlzQ29udmVyZ2VkKCkpXG4gICAgICB7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuXG4gICAgICB0aGlzLmNvb2xpbmdGYWN0b3IgPSB0aGlzLmluaXRpYWxDb29saW5nRmFjdG9yICpcbiAgICAgICAgICAgICAgKCh0aGlzLm1heEl0ZXJhdGlvbnMgLSB0aGlzLnRvdGFsSXRlcmF0aW9ucykgLyB0aGlzLm1heEl0ZXJhdGlvbnMpO1xuICAgICAgYW5pbWF0aW9uUGVyaW9kID0gTWF0aC5jZWlsKGluaXRpYWxBbmltYXRpb25QZXJpb2QgKiBNYXRoLnNxcnQodGhpcy5jb29saW5nRmFjdG9yKSk7XG5cbiAgICB9XG4gICAgdGhpcy50b3RhbERpc3BsYWNlbWVudCA9IDA7XG4gICAgdGhpcy5ncmFwaE1hbmFnZXIudXBkYXRlQm91bmRzKCk7XG4gICAgdGhpcy5jYWxjU3ByaW5nRm9yY2VzKCk7XG4gICAgdGhpcy5jYWxjUmVwdWxzaW9uRm9yY2VzKCk7XG4gICAgdGhpcy5jYWxjR3Jhdml0YXRpb25hbEZvcmNlcygpO1xuICAgIHRoaXMubW92ZU5vZGVzKCk7XG4gICAgdGhpcy5hbmltYXRlKCk7XG4gICAgaWYgKEZETGF5b3V0Q29uc3RhbnRzLkFOSU1BVEUgPT09ICdkdXJpbmcnICYmIHRoaXMudG90YWxJdGVyYXRpb25zICUgYW5pbWF0aW9uUGVyaW9kID09IDApIHtcbiAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgMWU3OyBpKyspIHtcbiAgICAgICAgaWYgKChuZXcgRGF0ZSgpLmdldFRpbWUoKSAtIGxhc3RGcmFtZSkgPiAyNSkge1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgICBsYXN0RnJhbWUgPSBuZXcgRGF0ZSgpLmdldFRpbWUoKTtcbiAgICAgIHZhciBhbGxOb2RlcyA9IHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEFsbE5vZGVzKCk7XG4gICAgICB2YXIgcERhdGEgPSB7fTtcbiAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgYWxsTm9kZXMubGVuZ3RoOyBpKyspIHtcbiAgICAgICAgdmFyIHJlY3QgPSBhbGxOb2Rlc1tpXS5yZWN0O1xuICAgICAgICB2YXIgaWQgPSBhbGxOb2Rlc1tpXS5pZDtcbiAgICAgICAgcERhdGFbaWRdID0ge1xuICAgICAgICAgIGlkOiBpZCxcbiAgICAgICAgICB4OiByZWN0LmdldENlbnRlclgoKSxcbiAgICAgICAgICB5OiByZWN0LmdldENlbnRlclkoKSxcbiAgICAgICAgICB3OiByZWN0LndpZHRoLFxuICAgICAgICAgIGg6IHJlY3QuaGVpZ2h0XG4gICAgICAgIH07XG4gICAgICB9XG4gICAgICBicm9hZGNhc3Qoe3BEYXRhOiBwRGF0YX0pO1xuICAgIH1cbiAgfVxuICB3aGlsZSAodGhpcy50b3RhbEl0ZXJhdGlvbnMgPCB0aGlzLm1heEl0ZXJhdGlvbnMpO1xuXG4gIHRoaXMuZ3JhcGhNYW5hZ2VyLnVwZGF0ZUJvdW5kcygpO1xufTtcblxuQ29TRUxheW91dC5wcm90b3R5cGUuY2FsY3VsYXRlTm9kZXNUb0FwcGx5R3Jhdml0YXRpb25UbyA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIG5vZGVMaXN0ID0gW107XG4gIHZhciBncmFwaDtcblxuICB2YXIgZ3JhcGhzID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0R3JhcGhzKCk7XG4gIHZhciBzaXplID0gZ3JhcGhzLmxlbmd0aDtcbiAgdmFyIGk7XG4gIGZvciAoaSA9IDA7IGkgPCBzaXplOyBpKyspXG4gIHtcbiAgICBncmFwaCA9IGdyYXBoc1tpXTtcblxuICAgIGdyYXBoLnVwZGF0ZUNvbm5lY3RlZCgpO1xuXG4gICAgaWYgKCFncmFwaC5pc0Nvbm5lY3RlZClcbiAgICB7XG4gICAgICBub2RlTGlzdCA9IG5vZGVMaXN0LmNvbmNhdChncmFwaC5nZXROb2RlcygpKTtcbiAgICB9XG4gIH1cblxuICB0aGlzLmdyYXBoTWFuYWdlci5zZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbihub2RlTGlzdCk7XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5jcmVhdGVCZW5kcG9pbnRzID0gZnVuY3Rpb24gKCkge1xuICB2YXIgZWRnZXMgPSBbXTtcbiAgZWRnZXMgPSBlZGdlcy5jb25jYXQodGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsRWRnZXMoKSk7XG4gIHZhciB2aXNpdGVkID0gbmV3IEhhc2hTZXQoKTtcbiAgdmFyIGk7XG4gIGZvciAoaSA9IDA7IGkgPCBlZGdlcy5sZW5ndGg7IGkrKylcbiAge1xuICAgIHZhciBlZGdlID0gZWRnZXNbaV07XG5cbiAgICBpZiAoIXZpc2l0ZWQuY29udGFpbnMoZWRnZSkpXG4gICAge1xuICAgICAgdmFyIHNvdXJjZSA9IGVkZ2UuZ2V0U291cmNlKCk7XG4gICAgICB2YXIgdGFyZ2V0ID0gZWRnZS5nZXRUYXJnZXQoKTtcblxuICAgICAgaWYgKHNvdXJjZSA9PSB0YXJnZXQpXG4gICAgICB7XG4gICAgICAgIGVkZ2UuZ2V0QmVuZHBvaW50cygpLnB1c2gobmV3IFBvaW50RCgpKTtcbiAgICAgICAgZWRnZS5nZXRCZW5kcG9pbnRzKCkucHVzaChuZXcgUG9pbnREKCkpO1xuICAgICAgICB0aGlzLmNyZWF0ZUR1bW15Tm9kZXNGb3JCZW5kcG9pbnRzKGVkZ2UpO1xuICAgICAgICB2aXNpdGVkLmFkZChlZGdlKTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgdmFyIGVkZ2VMaXN0ID0gW107XG5cbiAgICAgICAgZWRnZUxpc3QgPSBlZGdlTGlzdC5jb25jYXQoc291cmNlLmdldEVkZ2VMaXN0VG9Ob2RlKHRhcmdldCkpO1xuICAgICAgICBlZGdlTGlzdCA9IGVkZ2VMaXN0LmNvbmNhdCh0YXJnZXQuZ2V0RWRnZUxpc3RUb05vZGUoc291cmNlKSk7XG5cbiAgICAgICAgaWYgKCF2aXNpdGVkLmNvbnRhaW5zKGVkZ2VMaXN0WzBdKSlcbiAgICAgICAge1xuICAgICAgICAgIGlmIChlZGdlTGlzdC5sZW5ndGggPiAxKVxuICAgICAgICAgIHtcbiAgICAgICAgICAgIHZhciBrO1xuICAgICAgICAgICAgZm9yIChrID0gMDsgayA8IGVkZ2VMaXN0Lmxlbmd0aDsgaysrKVxuICAgICAgICAgICAge1xuICAgICAgICAgICAgICB2YXIgbXVsdGlFZGdlID0gZWRnZUxpc3Rba107XG4gICAgICAgICAgICAgIG11bHRpRWRnZS5nZXRCZW5kcG9pbnRzKCkucHVzaChuZXcgUG9pbnREKCkpO1xuICAgICAgICAgICAgICB0aGlzLmNyZWF0ZUR1bW15Tm9kZXNGb3JCZW5kcG9pbnRzKG11bHRpRWRnZSk7XG4gICAgICAgICAgICB9XG4gICAgICAgICAgfVxuICAgICAgICAgIHZpc2l0ZWQuYWRkQWxsKGxpc3QpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuXG4gICAgaWYgKHZpc2l0ZWQuc2l6ZSgpID09IGVkZ2VzLmxlbmd0aClcbiAgICB7XG4gICAgICBicmVhaztcbiAgICB9XG4gIH1cbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLnBvc2l0aW9uTm9kZXNSYWRpYWxseSA9IGZ1bmN0aW9uIChmb3Jlc3QpIHtcbiAgLy8gV2UgdGlsZSB0aGUgdHJlZXMgdG8gYSBncmlkIHJvdyBieSByb3c7IGZpcnN0IHRyZWUgc3RhcnRzIGF0ICgwLDApXG4gIHZhciBjdXJyZW50U3RhcnRpbmdQb2ludCA9IG5ldyBQb2ludCgwLCAwKTtcbiAgdmFyIG51bWJlck9mQ29sdW1ucyA9IE1hdGguY2VpbChNYXRoLnNxcnQoZm9yZXN0Lmxlbmd0aCkpO1xuICB2YXIgaGVpZ2h0ID0gMDtcbiAgdmFyIGN1cnJlbnRZID0gMDtcbiAgdmFyIGN1cnJlbnRYID0gMDtcbiAgdmFyIHBvaW50ID0gbmV3IFBvaW50RCgwLCAwKTtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IGZvcmVzdC5sZW5ndGg7IGkrKylcbiAge1xuICAgIGlmIChpICUgbnVtYmVyT2ZDb2x1bW5zID09IDApXG4gICAge1xuICAgICAgLy8gU3RhcnQgb2YgYSBuZXcgcm93LCBtYWtlIHRoZSB4IGNvb3JkaW5hdGUgMCwgaW5jcmVtZW50IHRoZVxuICAgICAgLy8geSBjb29yZGluYXRlIHdpdGggdGhlIG1heCBoZWlnaHQgb2YgdGhlIHByZXZpb3VzIHJvd1xuICAgICAgY3VycmVudFggPSAwO1xuICAgICAgY3VycmVudFkgPSBoZWlnaHQ7XG5cbiAgICAgIGlmIChpICE9IDApXG4gICAgICB7XG4gICAgICAgIGN1cnJlbnRZICs9IENvU0VDb25zdGFudHMuREVGQVVMVF9DT01QT05FTlRfU0VQRVJBVElPTjtcbiAgICAgIH1cblxuICAgICAgaGVpZ2h0ID0gMDtcbiAgICB9XG5cbiAgICB2YXIgdHJlZSA9IGZvcmVzdFtpXTtcblxuICAgIC8vIEZpbmQgdGhlIGNlbnRlciBvZiB0aGUgdHJlZVxuICAgIHZhciBjZW50ZXJOb2RlID0gTGF5b3V0LmZpbmRDZW50ZXJPZlRyZWUodHJlZSk7XG5cbiAgICAvLyBTZXQgdGhlIHN0YXJpbmcgcG9pbnQgb2YgdGhlIG5leHQgdHJlZVxuICAgIGN1cnJlbnRTdGFydGluZ1BvaW50LnggPSBjdXJyZW50WDtcbiAgICBjdXJyZW50U3RhcnRpbmdQb2ludC55ID0gY3VycmVudFk7XG5cbiAgICAvLyBEbyBhIHJhZGlhbCBsYXlvdXQgc3RhcnRpbmcgd2l0aCB0aGUgY2VudGVyXG4gICAgcG9pbnQgPVxuICAgICAgICAgICAgQ29TRUxheW91dC5yYWRpYWxMYXlvdXQodHJlZSwgY2VudGVyTm9kZSwgY3VycmVudFN0YXJ0aW5nUG9pbnQpO1xuXG4gICAgaWYgKHBvaW50LnkgPiBoZWlnaHQpXG4gICAge1xuICAgICAgaGVpZ2h0ID0gTWF0aC5mbG9vcihwb2ludC55KTtcbiAgICB9XG5cbiAgICBjdXJyZW50WCA9IE1hdGguZmxvb3IocG9pbnQueCArIENvU0VDb25zdGFudHMuREVGQVVMVF9DT01QT05FTlRfU0VQRVJBVElPTik7XG4gIH1cblxuICB0aGlzLnRyYW5zZm9ybShcbiAgICAgICAgICBuZXcgUG9pbnREKExheW91dENvbnN0YW50cy5XT1JMRF9DRU5URVJfWCAtIHBvaW50LnggLyAyLFxuICAgICAgICAgICAgICAgICAgTGF5b3V0Q29uc3RhbnRzLldPUkxEX0NFTlRFUl9ZIC0gcG9pbnQueSAvIDIpKTtcbn07XG5cbkNvU0VMYXlvdXQucmFkaWFsTGF5b3V0ID0gZnVuY3Rpb24gKHRyZWUsIGNlbnRlck5vZGUsIHN0YXJ0aW5nUG9pbnQpIHtcbiAgdmFyIHJhZGlhbFNlcCA9IE1hdGgubWF4KHRoaXMubWF4RGlhZ29uYWxJblRyZWUodHJlZSksXG4gICAgICAgICAgQ29TRUNvbnN0YW50cy5ERUZBVUxUX1JBRElBTF9TRVBBUkFUSU9OKTtcbiAgQ29TRUxheW91dC5icmFuY2hSYWRpYWxMYXlvdXQoY2VudGVyTm9kZSwgbnVsbCwgMCwgMzU5LCAwLCByYWRpYWxTZXApO1xuICB2YXIgYm91bmRzID0gTEdyYXBoLmNhbGN1bGF0ZUJvdW5kcyh0cmVlKTtcblxuICB2YXIgdHJhbnNmb3JtID0gbmV3IFRyYW5zZm9ybSgpO1xuICB0cmFuc2Zvcm0uc2V0RGV2aWNlT3JnWChib3VuZHMuZ2V0TWluWCgpKTtcbiAgdHJhbnNmb3JtLnNldERldmljZU9yZ1koYm91bmRzLmdldE1pblkoKSk7XG4gIHRyYW5zZm9ybS5zZXRXb3JsZE9yZ1goc3RhcnRpbmdQb2ludC54KTtcbiAgdHJhbnNmb3JtLnNldFdvcmxkT3JnWShzdGFydGluZ1BvaW50LnkpO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgdHJlZS5sZW5ndGg7IGkrKylcbiAge1xuICAgIHZhciBub2RlID0gdHJlZVtpXTtcbiAgICBub2RlLnRyYW5zZm9ybSh0cmFuc2Zvcm0pO1xuICB9XG5cbiAgdmFyIGJvdHRvbVJpZ2h0ID1cbiAgICAgICAgICBuZXcgUG9pbnREKGJvdW5kcy5nZXRNYXhYKCksIGJvdW5kcy5nZXRNYXhZKCkpO1xuXG4gIHJldHVybiB0cmFuc2Zvcm0uaW52ZXJzZVRyYW5zZm9ybVBvaW50KGJvdHRvbVJpZ2h0KTtcbn07XG5cbkNvU0VMYXlvdXQuYnJhbmNoUmFkaWFsTGF5b3V0ID0gZnVuY3Rpb24gKG5vZGUsIHBhcmVudE9mTm9kZSwgc3RhcnRBbmdsZSwgZW5kQW5nbGUsIGRpc3RhbmNlLCByYWRpYWxTZXBhcmF0aW9uKSB7XG4gIC8vIEZpcnN0LCBwb3NpdGlvbiB0aGlzIG5vZGUgYnkgZmluZGluZyBpdHMgYW5nbGUuXG4gIHZhciBoYWxmSW50ZXJ2YWwgPSAoKGVuZEFuZ2xlIC0gc3RhcnRBbmdsZSkgKyAxKSAvIDI7XG5cbiAgaWYgKGhhbGZJbnRlcnZhbCA8IDApXG4gIHtcbiAgICBoYWxmSW50ZXJ2YWwgKz0gMTgwO1xuICB9XG5cbiAgdmFyIG5vZGVBbmdsZSA9IChoYWxmSW50ZXJ2YWwgKyBzdGFydEFuZ2xlKSAlIDM2MDtcbiAgdmFyIHRldGEgPSAobm9kZUFuZ2xlICogSUdlb21ldHJ5LlRXT19QSSkgLyAzNjA7XG5cbiAgLy8gTWFrZSBwb2xhciB0byBqYXZhIGNvcmRpbmF0ZSBjb252ZXJzaW9uLlxuICB2YXIgY29zX3RldGEgPSBNYXRoLmNvcyh0ZXRhKTtcbiAgdmFyIHhfID0gZGlzdGFuY2UgKiBNYXRoLmNvcyh0ZXRhKTtcbiAgdmFyIHlfID0gZGlzdGFuY2UgKiBNYXRoLnNpbih0ZXRhKTtcblxuICBub2RlLnNldENlbnRlcih4XywgeV8pO1xuXG4gIC8vIFRyYXZlcnNlIGFsbCBuZWlnaGJvcnMgb2YgdGhpcyBub2RlIGFuZCByZWN1cnNpdmVseSBjYWxsIHRoaXNcbiAgLy8gZnVuY3Rpb24uXG4gIHZhciBuZWlnaGJvckVkZ2VzID0gW107XG4gIG5laWdoYm9yRWRnZXMgPSBuZWlnaGJvckVkZ2VzLmNvbmNhdChub2RlLmdldEVkZ2VzKCkpO1xuICB2YXIgY2hpbGRDb3VudCA9IG5laWdoYm9yRWRnZXMubGVuZ3RoO1xuXG4gIGlmIChwYXJlbnRPZk5vZGUgIT0gbnVsbClcbiAge1xuICAgIGNoaWxkQ291bnQtLTtcbiAgfVxuXG4gIHZhciBicmFuY2hDb3VudCA9IDA7XG5cbiAgdmFyIGluY0VkZ2VzQ291bnQgPSBuZWlnaGJvckVkZ2VzLmxlbmd0aDtcbiAgdmFyIHN0YXJ0SW5kZXg7XG5cbiAgdmFyIGVkZ2VzID0gbm9kZS5nZXRFZGdlc0JldHdlZW4ocGFyZW50T2ZOb2RlKTtcblxuICAvLyBJZiB0aGVyZSBhcmUgbXVsdGlwbGUgZWRnZXMsIHBydW5lIHRoZW0gdW50aWwgdGhlcmUgcmVtYWlucyBvbmx5IG9uZVxuICAvLyBlZGdlLlxuICB3aGlsZSAoZWRnZXMubGVuZ3RoID4gMSlcbiAge1xuICAgIC8vbmVpZ2hib3JFZGdlcy5yZW1vdmUoZWRnZXMucmVtb3ZlKDApKTtcbiAgICB2YXIgdGVtcCA9IGVkZ2VzWzBdO1xuICAgIGVkZ2VzLnNwbGljZSgwLCAxKTtcbiAgICB2YXIgaW5kZXggPSBuZWlnaGJvckVkZ2VzLmluZGV4T2YodGVtcCk7XG4gICAgaWYgKGluZGV4ID49IDApIHtcbiAgICAgIG5laWdoYm9yRWRnZXMuc3BsaWNlKGluZGV4LCAxKTtcbiAgICB9XG4gICAgaW5jRWRnZXNDb3VudC0tO1xuICAgIGNoaWxkQ291bnQtLTtcbiAgfVxuXG4gIGlmIChwYXJlbnRPZk5vZGUgIT0gbnVsbClcbiAge1xuICAgIC8vYXNzZXJ0IGVkZ2VzLmxlbmd0aCA9PSAxO1xuICAgIHN0YXJ0SW5kZXggPSAobmVpZ2hib3JFZGdlcy5pbmRleE9mKGVkZ2VzWzBdKSArIDEpICUgaW5jRWRnZXNDb3VudDtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICBzdGFydEluZGV4ID0gMDtcbiAgfVxuXG4gIHZhciBzdGVwQW5nbGUgPSBNYXRoLmFicyhlbmRBbmdsZSAtIHN0YXJ0QW5nbGUpIC8gY2hpbGRDb3VudDtcblxuICBmb3IgKHZhciBpID0gc3RhcnRJbmRleDtcbiAgICAgICAgICBicmFuY2hDb3VudCAhPSBjaGlsZENvdW50O1xuICAgICAgICAgIGkgPSAoKytpKSAlIGluY0VkZ2VzQ291bnQpXG4gIHtcbiAgICB2YXIgY3VycmVudE5laWdoYm9yID1cbiAgICAgICAgICAgIG5laWdoYm9yRWRnZXNbaV0uZ2V0T3RoZXJFbmQobm9kZSk7XG5cbiAgICAvLyBEb24ndCBiYWNrIHRyYXZlcnNlIHRvIHJvb3Qgbm9kZSBpbiBjdXJyZW50IHRyZWUuXG4gICAgaWYgKGN1cnJlbnROZWlnaGJvciA9PSBwYXJlbnRPZk5vZGUpXG4gICAge1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgdmFyIGNoaWxkU3RhcnRBbmdsZSA9XG4gICAgICAgICAgICAoc3RhcnRBbmdsZSArIGJyYW5jaENvdW50ICogc3RlcEFuZ2xlKSAlIDM2MDtcbiAgICB2YXIgY2hpbGRFbmRBbmdsZSA9IChjaGlsZFN0YXJ0QW5nbGUgKyBzdGVwQW5nbGUpICUgMzYwO1xuXG4gICAgQ29TRUxheW91dC5icmFuY2hSYWRpYWxMYXlvdXQoY3VycmVudE5laWdoYm9yLFxuICAgICAgICAgICAgbm9kZSxcbiAgICAgICAgICAgIGNoaWxkU3RhcnRBbmdsZSwgY2hpbGRFbmRBbmdsZSxcbiAgICAgICAgICAgIGRpc3RhbmNlICsgcmFkaWFsU2VwYXJhdGlvbiwgcmFkaWFsU2VwYXJhdGlvbik7XG5cbiAgICBicmFuY2hDb3VudCsrO1xuICB9XG59O1xuXG5Db1NFTGF5b3V0Lm1heERpYWdvbmFsSW5UcmVlID0gZnVuY3Rpb24gKHRyZWUpIHtcbiAgdmFyIG1heERpYWdvbmFsID0gSW50ZWdlci5NSU5fVkFMVUU7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCB0cmVlLmxlbmd0aDsgaSsrKVxuICB7XG4gICAgdmFyIG5vZGUgPSB0cmVlW2ldO1xuICAgIHZhciBkaWFnb25hbCA9IG5vZGUuZ2V0RGlhZ29uYWwoKTtcblxuICAgIGlmIChkaWFnb25hbCA+IG1heERpYWdvbmFsKVxuICAgIHtcbiAgICAgIG1heERpYWdvbmFsID0gZGlhZ29uYWw7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIG1heERpYWdvbmFsO1xufTtcblxuQ29TRUxheW91dC5wcm90b3R5cGUuY2FsY1JlcHVsc2lvblJhbmdlID0gZnVuY3Rpb24gKCkge1xuICAvLyBmb3JtdWxhIGlzIDIgeCAobGV2ZWwgKyAxKSB4IGlkZWFsRWRnZUxlbmd0aFxuICByZXR1cm4gKDIgKiAodGhpcy5sZXZlbCArIDEpICogdGhpcy5pZGVhbEVkZ2VMZW5ndGgpO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSBDb1NFTGF5b3V0O1xuIiwidmFyIEZETGF5b3V0Tm9kZSA9IHJlcXVpcmUoJy4vRkRMYXlvdXROb2RlJyk7XG5cbmZ1bmN0aW9uIENvU0VOb2RlKGdtLCBsb2MsIHNpemUsIHZOb2RlKSB7XG4gIEZETGF5b3V0Tm9kZS5jYWxsKHRoaXMsIGdtLCBsb2MsIHNpemUsIHZOb2RlKTtcbn1cblxuXG5Db1NFTm9kZS5wcm90b3R5cGUgPSBPYmplY3QuY3JlYXRlKEZETGF5b3V0Tm9kZS5wcm90b3R5cGUpO1xuZm9yICh2YXIgcHJvcCBpbiBGRExheW91dE5vZGUpIHtcbiAgQ29TRU5vZGVbcHJvcF0gPSBGRExheW91dE5vZGVbcHJvcF07XG59XG5cbkNvU0VOb2RlLnByb3RvdHlwZS5tb3ZlID0gZnVuY3Rpb24gKClcbntcbiAgdmFyIGxheW91dCA9IHRoaXMuZ3JhcGhNYW5hZ2VyLmdldExheW91dCgpO1xuICB0aGlzLmRpc3BsYWNlbWVudFggPSBsYXlvdXQuY29vbGluZ0ZhY3RvciAqXG4gICAgICAgICAgKHRoaXMuc3ByaW5nRm9yY2VYICsgdGhpcy5yZXB1bHNpb25Gb3JjZVggKyB0aGlzLmdyYXZpdGF0aW9uRm9yY2VYKTtcbiAgdGhpcy5kaXNwbGFjZW1lbnRZID0gbGF5b3V0LmNvb2xpbmdGYWN0b3IgKlxuICAgICAgICAgICh0aGlzLnNwcmluZ0ZvcmNlWSArIHRoaXMucmVwdWxzaW9uRm9yY2VZICsgdGhpcy5ncmF2aXRhdGlvbkZvcmNlWSk7XG5cblxuICBpZiAoTWF0aC5hYnModGhpcy5kaXNwbGFjZW1lbnRYKSA+IGxheW91dC5jb29saW5nRmFjdG9yICogbGF5b3V0Lm1heE5vZGVEaXNwbGFjZW1lbnQpXG4gIHtcbiAgICB0aGlzLmRpc3BsYWNlbWVudFggPSBsYXlvdXQuY29vbGluZ0ZhY3RvciAqIGxheW91dC5tYXhOb2RlRGlzcGxhY2VtZW50ICpcbiAgICAgICAgICAgIElNYXRoLnNpZ24odGhpcy5kaXNwbGFjZW1lbnRYKTtcbiAgfVxuXG4gIGlmIChNYXRoLmFicyh0aGlzLmRpc3BsYWNlbWVudFkpID4gbGF5b3V0LmNvb2xpbmdGYWN0b3IgKiBsYXlvdXQubWF4Tm9kZURpc3BsYWNlbWVudClcbiAge1xuICAgIHRoaXMuZGlzcGxhY2VtZW50WSA9IGxheW91dC5jb29saW5nRmFjdG9yICogbGF5b3V0Lm1heE5vZGVEaXNwbGFjZW1lbnQgKlxuICAgICAgICAgICAgSU1hdGguc2lnbih0aGlzLmRpc3BsYWNlbWVudFkpO1xuICB9XG5cbiAgLy8gYSBzaW1wbGUgbm9kZSwganVzdCBtb3ZlIGl0XG4gIGlmICh0aGlzLmNoaWxkID09IG51bGwpXG4gIHtcbiAgICB0aGlzLm1vdmVCeSh0aGlzLmRpc3BsYWNlbWVudFgsIHRoaXMuZGlzcGxhY2VtZW50WSk7XG4gIH1cbiAgLy8gYW4gZW1wdHkgY29tcG91bmQgbm9kZSwgYWdhaW4ganVzdCBtb3ZlIGl0XG4gIGVsc2UgaWYgKHRoaXMuY2hpbGQuZ2V0Tm9kZXMoKS5sZW5ndGggPT0gMClcbiAge1xuICAgIHRoaXMubW92ZUJ5KHRoaXMuZGlzcGxhY2VtZW50WCwgdGhpcy5kaXNwbGFjZW1lbnRZKTtcbiAgfVxuICAvLyBub24tZW1wdHkgY29tcG91bmQgbm9kZSwgcHJvcG9nYXRlIG1vdmVtZW50IHRvIGNoaWxkcmVuIGFzIHdlbGxcbiAgZWxzZVxuICB7XG4gICAgdGhpcy5wcm9wb2dhdGVEaXNwbGFjZW1lbnRUb0NoaWxkcmVuKHRoaXMuZGlzcGxhY2VtZW50WCxcbiAgICAgICAgICAgIHRoaXMuZGlzcGxhY2VtZW50WSk7XG4gIH1cblxuICBsYXlvdXQudG90YWxEaXNwbGFjZW1lbnQgKz1cbiAgICAgICAgICBNYXRoLmFicyh0aGlzLmRpc3BsYWNlbWVudFgpICsgTWF0aC5hYnModGhpcy5kaXNwbGFjZW1lbnRZKTtcblxuICB0aGlzLnNwcmluZ0ZvcmNlWCA9IDA7XG4gIHRoaXMuc3ByaW5nRm9yY2VZID0gMDtcbiAgdGhpcy5yZXB1bHNpb25Gb3JjZVggPSAwO1xuICB0aGlzLnJlcHVsc2lvbkZvcmNlWSA9IDA7XG4gIHRoaXMuZ3Jhdml0YXRpb25Gb3JjZVggPSAwO1xuICB0aGlzLmdyYXZpdGF0aW9uRm9yY2VZID0gMDtcbiAgdGhpcy5kaXNwbGFjZW1lbnRYID0gMDtcbiAgdGhpcy5kaXNwbGFjZW1lbnRZID0gMDtcbn07XG5cbkNvU0VOb2RlLnByb3RvdHlwZS5wcm9wb2dhdGVEaXNwbGFjZW1lbnRUb0NoaWxkcmVuID0gZnVuY3Rpb24gKGRYLCBkWSlcbntcbiAgdmFyIG5vZGVzID0gdGhpcy5nZXRDaGlsZCgpLmdldE5vZGVzKCk7XG4gIHZhciBub2RlO1xuICBmb3IgKHZhciBpID0gMDsgaSA8IG5vZGVzLmxlbmd0aDsgaSsrKVxuICB7XG4gICAgbm9kZSA9IG5vZGVzW2ldO1xuICAgIGlmIChub2RlLmdldENoaWxkKCkgPT0gbnVsbClcbiAgICB7XG4gICAgICBub2RlLm1vdmVCeShkWCwgZFkpO1xuICAgICAgbm9kZS5kaXNwbGFjZW1lbnRYICs9IGRYO1xuICAgICAgbm9kZS5kaXNwbGFjZW1lbnRZICs9IGRZO1xuICAgIH1cbiAgICBlbHNlXG4gICAge1xuICAgICAgbm9kZS5wcm9wb2dhdGVEaXNwbGFjZW1lbnRUb0NoaWxkcmVuKGRYLCBkWSk7XG4gICAgfVxuICB9XG59O1xuXG5Db1NFTm9kZS5wcm90b3R5cGUuc2V0UHJlZDEgPSBmdW5jdGlvbiAocHJlZDEpXG57XG4gIHRoaXMucHJlZDEgPSBwcmVkMTtcbn07XG5cbkNvU0VOb2RlLnByb3RvdHlwZS5nZXRQcmVkMSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiBwcmVkMTtcbn07XG5cbkNvU0VOb2RlLnByb3RvdHlwZS5nZXRQcmVkMiA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiBwcmVkMjtcbn07XG5cbkNvU0VOb2RlLnByb3RvdHlwZS5zZXROZXh0ID0gZnVuY3Rpb24gKG5leHQpXG57XG4gIHRoaXMubmV4dCA9IG5leHQ7XG59O1xuXG5Db1NFTm9kZS5wcm90b3R5cGUuZ2V0TmV4dCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiBuZXh0O1xufTtcblxuQ29TRU5vZGUucHJvdG90eXBlLnNldFByb2Nlc3NlZCA9IGZ1bmN0aW9uIChwcm9jZXNzZWQpXG57XG4gIHRoaXMucHJvY2Vzc2VkID0gcHJvY2Vzc2VkO1xufTtcblxuQ29TRU5vZGUucHJvdG90eXBlLmlzUHJvY2Vzc2VkID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHByb2Nlc3NlZDtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gQ29TRU5vZGU7XG4iLCJ2YXIgVmlzaWJpbGl0eUVkZ2UgPSByZXF1aXJlKCcuL1Zpc2liaWxpdHlFZGdlJyk7XG52YXIgVmlzaWJpbGl0eUdyYXBoID0gcmVxdWlyZSgnLi9WaXNpYmlsaXR5R3JhcGgnKTtcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xuXG5mdW5jdGlvbiBDb21wYWN0aW9uKHZlcnRpY2VzKSBcbntcbiAgICB0aGlzLm9yZGVyZWROb2RlTGlzdCA9IFtdIC8qQXJyYXlMaXN0PFNiZ25QRE5vZGU+KCkqLztcbiAgICB0aGlzLnZlcnRpY2VzID0gdmVydGljZXM7XG4gICAgXG4gICAgdGhpcy52aXNHcmFwaCA9IG51bGw7XG4gICAgdGhpcy5kaXJlY3Rpb24gPSBudWxsO1xufVxuXG5Db21wYWN0aW9uLnByb3RvdHlwZS5Db21wYWN0aW9uRGlyZWN0aW9uRW51bSA9IFxue1xuICAgIFZFUlRJQ0FMIDogMCwgXG4gICAgSE9SSVpPTlRBTCA6IDFcbn07XG5cbi8qKlxuKiBUd28gdGltZXMgZG8gdGhlIGZvbGxvd2luZzogKGZpcnN0IGZvciB2ZXJ0aWNhbCwgc2Vjb25kIGhvcml6b250YWwpIEZpcnN0XG4qIGNyZWF0ZSBhIHZpc2liaWxpdHkgZ3JhcGggZm9yIHRoZSBnaXZlbiBlbGVtZW50cy4gVGhlIHZpc2liaWxpdHkgZ3JhcGggaXNcbiogYWx3YXlzIGEgREFHLCBzbyBwZXJmb3JtIGEgdG9wb2xvZ2ljYWwgc29ydCBvbiB0aGUgZWxlbWVudHMgKHRoZSBub2RlXG4qIHRoYXQgaGFzIGluLWRlZ3JlZSAwIGNvbWVzIGZpcnN0KSwgcGVyZm9ybSBjb21wYWN0aW9uLlxuKi9cbkNvbXBhY3Rpb24ucHJvdG90eXBlLnBlcmZvcm0gPSBmdW5jdGlvbiAoKVxue1xuICAgIHRoaXMuYWxnb3JpdGhtQm9keSh0aGlzLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKTtcbiAgICB0aGlzLnJlbW92ZVZpc2liaWxpdHlFZGdlcygpO1xuXG4gICAgdGhpcy5hbGdvcml0aG1Cb2R5KHRoaXMuQ29tcGFjdGlvbkRpcmVjdGlvbkVudW0uSE9SSVpPTlRBTCk7XG4gICAgdGhpcy5yZW1vdmVWaXNpYmlsaXR5RWRnZXMoKTtcblxufTtcblxuQ29tcGFjdGlvbi5wcm90b3R5cGUucmVtb3ZlVmlzaWJpbGl0eUVkZ2VzID0gZnVuY3Rpb24gKClcbntcbiAgICB2YXIgbnVtT2ZWZXJ0aWNlcyA9IHRoaXMudmVydGljZXMubGVuZ3RoO1xuICAgIGZvcih2YXIgaT0wOyBpPG51bU9mVmVydGljZXM7IGkrKylcbiAgICB7XG4gICAgICAgIHZhciBzYmduTm9kZSA9IHRoaXMudmVydGljZXNbaV07XG5cbiAgICAgICAgZm9yKHZhciBqID0gMDsgaiA8IHNiZ25Ob2RlLmdldEVkZ2VzKCkubGVuZ3RoOyBqKyspXG4gICAgICAgIHtcbiAgICAgICAgICAgIHZhciBvYmpFZGdlID0gc2Jnbk5vZGUuZ2V0RWRnZXMoKVtqXTtcblxuICAgICAgICAgICAgaWYoIG9iakVkZ2UgaW5zdGFuY2VvZiBWaXNpYmlsaXR5RWRnZSlcbiAgICAgICAgICAgIHtcbiAgICAgICAgICAgICAgICBzYmduTm9kZS5nZXRFZGdlcygpLnNwbGljZShpLCAxKTtcbiAgICAgICAgICAgICAgICBpLS07XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICB9XHRcbn07XG5cbkNvbXBhY3Rpb24ucHJvdG90eXBlLmFsZ29yaXRobUJvZHkgPSBmdW5jdGlvbiAoZGlyZWN0aW9uKVxue1xuICAgIHRoaXMuZGlyZWN0aW9uID0gZGlyZWN0aW9uO1xuICAgIHRoaXMudmlzR3JhcGggPSBuZXcgVmlzaWJpbGl0eUdyYXBoKG51bGwsIG51bGwsIG51bGwpO1xuXG4gICAgLy8gY29uc3RydWN0IGEgdmlzaWJpbGl0eSBncmFwaCBnaXZlbiB0aGUgZGlyZWN0aW9uIGFuZCB2ZXJ0aWNlc1xuICAgIHRoaXMudmlzR3JhcGguY29uc3RydWN0KHRoaXMuZGlyZWN0aW9uLCB2ZXJ0aWNlcyk7XG5cbiAgICBpZiAodGhpcy52aXNHcmFwaC5nZXRFZGdlcygpLmxlbmd0aCA+IDApXG4gICAge1xuICAgICAgICB0aGlzLnRvcG9sb2dpY2FsbHlTb3J0KCk7XG4gICAgICAgIHRoaXMuY29tcGFjdEVsZW1lbnRzKCk7XG4gICAgfVxuXG4gICAgLy8gcG9zaXRpb25zIG9mIHRoZSB2ZXJ0aWNlcyBoYXMgY2hhbmdlZC4gVXBkYXRlIHRoZW0uXG4gICAgdGhpcy52ZXJ0aWNlcyA9IHRoaXMudmlzR3JhcGguZ2V0Tm9kZXMoKTtcbn07XG5cbi8qKlxuKiBQZXJmb3JtIGEgREZTIG9uIHRoZSBnaXZlbiBncmFwaCBub2RlcyBhbmQgdGhlbiBvdXRwdXQgdGhlIG5vZGVzIGluXG4qIHJldmVyc2Ugb3JkZXIuXG4qL1xuQ29tcGFjdGlvbi5wcm90b3R5cGUudG9wb2xvZ2ljYWxseVNvcnQgPSBmdW5jdGlvbiAoKVxue1xuICAgIC8vIGVuc3VyZSB0aGF0IHRoZSB2ZXJ0aWNlcyBoYXZlIG5vdCBiZWVuIG1hcmtlZCBhcyB2aXNpdGVkLlxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy52aXNHcmFwaC5nZXROb2RlcygpLmxlbmd0aDsgaSsrKVxuICAgIHtcbiAgICAgICAgdmFyIHMgPSB0aGlzLnZpc0dyYXBoLmdldE5vZGVzKClbaV07XG4gICAgICAgIHMudmlzaXRlZCA9IGZhbHNlO1xuICAgIH1cblxuICAgIC8vIGVuc3VyZSB0aGF0IHRoZSBsaXN0IGlzIGVtcHR5XG4gICAgdGhpcy5vcmRlcmVkTm9kZUxpc3QgPSBbXTtcbiAgICB0aGlzLkRGUygpO1xuICAgIHRoaXMub3JkZXJlZE5vZGVMaXN0ID0gdGhpcy5yZXZlcnNlTGlzdCh0aGlzLm9yZGVyZWROb2RlTGlzdCk7XG59O1xuXG5Db21wYWN0aW9uLnByb3RvdHlwZS5ERlMgPSBmdW5jdGlvbiAoKVxue1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy52aXNHcmFwaC5nZXROb2RlcygpLmxlbmd0aDsgaSsrKVxuICAgIHtcbiAgICAgICAgdmFyIHMgPSB0aGlzLnZpc0dyYXBoLmdldE5vZGVzKClbaV07XG4gICAgICAgIGlmICghcy52aXNpdGVkKVxuICAgICAgICB7XG4gICAgICAgICAgICB0aGlzLkRGU19WaXNpdChzKTtcbiAgICAgICAgfVxuICAgIH1cbn07XG5cbkNvbXBhY3Rpb24ucHJvdG90eXBlLkRGU19WaXNpdCA9IGZ1bmN0aW9uIChzKVxue1xuICAgIHZhciBuZWlnaGJvcnMgPSBzLmdldENoaWxkcmVuTmVpZ2hib3JzKG51bGwpO1xuXG4gICAgaWYgKG5laWdoYm9ycy5sZW5ndGggPT09IDApXG4gICAge1xuICAgICAgICBzLnZpc2l0ZWQgPSB0cnVlO1xuICAgICAgICB0aGlzLm9yZGVyZWROb2RlTGlzdC5wdXNoKHMpO1xuICAgICAgICByZXR1cm47XG4gICAgfVxuXG4gICAgdmFyIG51bU9mTmVpZ2hib3VycyA9IG5laWdoYm9ycy5sZW5ndGg7XG4gICAgZm9yICh2YXIgaT0wOyBpPG51bU9mTmVpZ2hib3VyczsgaSsrKVxuICAgIHtcbiAgICAgICAgaWYgKCFuZWlnaGJvcnNbaV0udmlzaXRlZClcbiAgICAgICAge1xuICAgICAgICAgICAgdGhpcy5ERlNfVmlzaXQobmVpZ2hib3JzW2ldKTtcbiAgICAgICAgfVxuICAgIH1cblxuICAgIHMudmlzaXRlZCA9IHRydWU7XG4gICAgdGhpcy5vcmRlcmVkTm9kZUxpc3QucHVzaChzKTtcbn07XG5cbi8qKlxuKiBSZXZlcnNlIHRoZSBlbGVtZW50IG9yZGVyIG9mIGEgZ2l2ZW4gbGlzdFxuKi9cbkNvbXBhY3Rpb24ucHJvdG90eXBlLnJldmVyc2VMaXN0ID0gZnVuY3Rpb24gKG9yaWdpbmFsTGlzdClcbntcbiAgICB2YXIgcmV2ZXJzZU91dHB1dCA9IFtdO1xuICAgIGZvciAodmFyIGkgPSBvcmlnaW5hbExpc3QubGVuZ3RoIC0gMTsgaSA+PSAwOyBpLS0pXG4gICAge1xuICAgICAgICByZXZlcnNlT3V0cHV0LnB1c2gob3JpZ2luYWxMaXN0W2ldKTtcbiAgICB9XG5cbiAgICByZXR1cm4gcmV2ZXJzZU91dHB1dDtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCB2aXNpdHMgdGhlIGxpc3QgdGhhdCBpcyB0aGUgcmVzdWx0IG9mIHRvcG9sb2dpY2FsIHNvcnQuIEZvclxuKiBlYWNoIG5vZGUgaW4gdGhhdCBsaXN0LCBpdCBsb29rcyBmb3IgaXRzIGluY29taW5nIGVkZ2VzIGFuZCBmaW5kcyB0aGVcbiogc2hvcnRlc3Qgb25lLiBUcmFuc2xhdGVzIHRoZSBub2RlIHdydCB0aGUgc2hvcnRlc3QgZWRnZS5cbiovXG5Db21wYWN0aW9uLnByb3RvdHlwZS5jb21wYWN0RWxlbWVudHMgPSBmdW5jdGlvbiAoKVxue1xuICAgIHZhciBkaXN0YW5jZSA9IDAuMDtcbiAgICBcbiAgICB2YXIgb3JkZXJlZE5vZGVMaXN0TGVuZ3RoID0gdGhpcy5vcmRlcmVkTm9kZUxpc3QubGVuZ3RoO1xuICAgIGZvciAodmFyIGk9MDsgaTxvcmRlcmVkTm9kZUxpc3RMZW5ndGg7IGkrKylcbiAgICB7XG4gICAgICAgIHZhciBzYmduUEROb2RlID0gdGhpcy5vcmRlcmVkTm9kZUxpc3RbaV07XG4gICAgICAgIFxuICAgICAgICAvLyBmaW5kIHNob3J0ZXN0IGluY29taW5nIGVkZ2VcbiAgICAgICAgdmFyIGVkZ2UgPSB0aGlzLnZpc0dyYXBoLmZpbmRTaG9ydGVzdEVkZ2Uoc2JnblBETm9kZSk7XG5cbiAgICAgICAgaWYgKGVkZ2UgIT0gbnVsbClcbiAgICAgICAge1xuICAgICAgICAgICAgZGlzdGFuY2UgPSBlZGdlLmdldExlbmd0aCgpO1xuXG4gICAgICAgICAgICBpZiAodGhpcy5kaXJlY3Rpb24gPT09IHRoaXMuQ29tcGFjdGlvbkRpcmVjdGlvbkVudW0uVkVSVElDQUwpXG4gICAgICAgICAgICB7XG4gICAgICAgICAgICAgICAgLy8gYnJpbmcgdGhlIG5vZGUgY2xvc2VyIHRvIHRoZSBzb3VyY2Ugbm9kZSBhbmQgcmVzcGVjdCB0aGVcbiAgICAgICAgICAgICAgICAvLyBidWZmZXIuXG4gICAgICAgICAgICAgICAgaWYgKGRpc3RhbmNlID4gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX1ZFUlRJQ0FMX0JVRkZFUilcbiAgICAgICAgICAgICAgICB7XG4gICAgICAgICAgICAgICAgICAgIHNiZ25QRE5vZGUuc2V0TG9jYXRpb24oXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgc2JnblBETm9kZS5nZXRMZWZ0KCksXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKHNiZ25QRE5vZGUuZ2V0VG9wKCkgLSBcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIChkaXN0YW5jZSAtIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVIpKSk7XG4gICAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgICAgIGVsc2VcbiAgICAgICAgICAgICAgICB7XG4gICAgICAgICAgICAgICAgICAgIHNiZ25QRE5vZGUuc2V0TG9jYXRpb24oXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgc2JnblBETm9kZS5nZXRMZWZ0KCksIFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGVkZ2UuZ2V0T3RoZXJFbmQoc2JnblBETm9kZSkuZ2V0Qm90dG9tKClcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX1ZFUlRJQ0FMX0JVRkZFUik7XG4gICAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuICAgICAgICAgICAgZWxzZSBpZiAodGhpcy5kaXJlY3Rpb24gPT09IHRoaXMuQ29tcGFjdGlvbkRpcmVjdGlvbkVudW0uSE9SSVpPTlRBTClcbiAgICAgICAgICAgIHtcbiAgICAgICAgICAgICAgICBpZiAoZGlzdGFuY2UgPiBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fSE9SSVpPTlRBTF9CVUZGRVIpXG4gICAgICAgICAgICAgICAge1xuICAgICAgICAgICAgICAgICAgICBzYmduUEROb2RlLnNldExvY2F0aW9uKFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIChzYmduUEROb2RlLmdldExlZnQoKSAtIChkaXN0YW5jZSAtIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUikpLFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHNiZ25QRE5vZGUuZ2V0VG9wKCkpO1xuICAgICAgICAgICAgICAgIH1cbiAgICAgICAgICAgICAgICBlbHNlXG4gICAgICAgICAgICAgICAge1xuICAgICAgICAgICAgICAgICAgICBzYmduUEROb2RlLnNldExvY2F0aW9uKFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGVkZ2UuZ2V0T3RoZXJFbmQoc2JnblBETm9kZSkuZ2V0UmlnaHQoKVxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUixcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBzYmduUEROb2RlLmdldFRvcCgpKTtcbiAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICB9XG4gICAgICAgIH1cbiAgICB9XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IENvbXBhY3Rpb247XG5cblxuIiwiZnVuY3Rpb24gRGltZW5zaW9uRCh3aWR0aCwgaGVpZ2h0KSB7XG4gIHRoaXMud2lkdGggPSAwO1xuICB0aGlzLmhlaWdodCA9IDA7XG4gIGlmICh3aWR0aCAhPT0gbnVsbCAmJiBoZWlnaHQgIT09IG51bGwpIHtcbiAgICB0aGlzLmhlaWdodCA9IGhlaWdodDtcbiAgICB0aGlzLndpZHRoID0gd2lkdGg7XG4gIH1cbn1cblxuRGltZW5zaW9uRC5wcm90b3R5cGUuZ2V0V2lkdGggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy53aWR0aDtcbn07XG5cbkRpbWVuc2lvbkQucHJvdG90eXBlLnNldFdpZHRoID0gZnVuY3Rpb24gKHdpZHRoKVxue1xuICB0aGlzLndpZHRoID0gd2lkdGg7XG59O1xuXG5EaW1lbnNpb25ELnByb3RvdHlwZS5nZXRIZWlnaHQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5oZWlnaHQ7XG59O1xuXG5EaW1lbnNpb25ELnByb3RvdHlwZS5zZXRIZWlnaHQgPSBmdW5jdGlvbiAoaGVpZ2h0KVxue1xuICB0aGlzLmhlaWdodCA9IGhlaWdodDtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gRGltZW5zaW9uRDtcbiIsInZhciBMYXlvdXQgPSByZXF1aXJlKCcuL0xheW91dCcpO1xudmFyIEZETGF5b3V0Q29uc3RhbnRzID0gcmVxdWlyZSgnLi9GRExheW91dENvbnN0YW50cycpO1xuXG5mdW5jdGlvbiBGRExheW91dCgpIHtcbiAgTGF5b3V0LmNhbGwodGhpcyk7XG5cbiAgdGhpcy51c2VTbWFydElkZWFsRWRnZUxlbmd0aENhbGN1bGF0aW9uID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9VU0VfU01BUlRfSURFQUxfRURHRV9MRU5HVEhfQ0FMQ1VMQVRJT047XG4gIHRoaXMuaWRlYWxFZGdlTGVuZ3RoID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9FREdFX0xFTkdUSDtcbiAgdGhpcy5zcHJpbmdDb25zdGFudCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfU1BSSU5HX1NUUkVOR1RIO1xuICB0aGlzLnJlcHVsc2lvbkNvbnN0YW50ID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9SRVBVTFNJT05fU1RSRU5HVEg7XG4gIHRoaXMuZ3Jhdml0eUNvbnN0YW50ID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1NUUkVOR1RIO1xuICB0aGlzLmNvbXBvdW5kR3Jhdml0eUNvbnN0YW50ID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1NUUkVOR1RIO1xuICB0aGlzLmdyYXZpdHlSYW5nZUZhY3RvciA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9SQU5HRV9GQUNUT1I7XG4gIHRoaXMuY29tcG91bmRHcmF2aXR5UmFuZ2VGYWN0b3IgPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0NPTVBPVU5EX0dSQVZJVFlfUkFOR0VfRkFDVE9SO1xuICB0aGlzLmRpc3BsYWNlbWVudFRocmVzaG9sZFBlck5vZGUgPSAoMy4wICogRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9FREdFX0xFTkdUSCkgLyAxMDA7XG4gIHRoaXMuY29vbGluZ0ZhY3RvciA9IDEuMDtcbiAgdGhpcy5pbml0aWFsQ29vbGluZ0ZhY3RvciA9IDEuMDtcbiAgdGhpcy50b3RhbERpc3BsYWNlbWVudCA9IDAuMDtcbiAgdGhpcy5vbGRUb3RhbERpc3BsYWNlbWVudCA9IDAuMDtcbiAgdGhpcy5tYXhJdGVyYXRpb25zID0gRkRMYXlvdXRDb25zdGFudHMuTUFYX0lURVJBVElPTlM7XG59XG5cbkZETGF5b3V0LnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoTGF5b3V0LnByb3RvdHlwZSk7XG5cbmZvciAodmFyIHByb3AgaW4gTGF5b3V0KSB7XG4gIEZETGF5b3V0W3Byb3BdID0gTGF5b3V0W3Byb3BdO1xufVxuXG5GRExheW91dC5wcm90b3R5cGUuaW5pdFBhcmFtZXRlcnMgPSBmdW5jdGlvbiAoKSB7XG4gIExheW91dC5wcm90b3R5cGUuaW5pdFBhcmFtZXRlcnMuY2FsbCh0aGlzLCBhcmd1bWVudHMpO1xuXG4gIGlmICh0aGlzLmxheW91dFF1YWxpdHkgPT0gTGF5b3V0Q29uc3RhbnRzLkRSQUZUX1FVQUxJVFkpXG4gIHtcbiAgICB0aGlzLmRpc3BsYWNlbWVudFRocmVzaG9sZFBlck5vZGUgKz0gMC4zMDtcbiAgICB0aGlzLm1heEl0ZXJhdGlvbnMgKj0gMC44O1xuICB9XG4gIGVsc2UgaWYgKHRoaXMubGF5b3V0UXVhbGl0eSA9PSBMYXlvdXRDb25zdGFudHMuUFJPT0ZfUVVBTElUWSlcbiAge1xuICAgIHRoaXMuZGlzcGxhY2VtZW50VGhyZXNob2xkUGVyTm9kZSAtPSAwLjMwO1xuICAgIHRoaXMubWF4SXRlcmF0aW9ucyAqPSAxLjI7XG4gIH1cblxuICB0aGlzLnRvdGFsSXRlcmF0aW9ucyA9IDA7XG4gIHRoaXMubm90QW5pbWF0ZWRJdGVyYXRpb25zID0gMDtcblxuLy8gICAgdGhpcy51c2VGUkdyaWRWYXJpYW50ID0gbGF5b3V0T3B0aW9uc1BhY2suc21hcnRSZXB1bHNpb25SYW5nZUNhbGM7XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuY2FsY0lkZWFsRWRnZUxlbmd0aHMgPSBmdW5jdGlvbiAoKSB7XG4gIHZhciBlZGdlO1xuICB2YXIgbGNhRGVwdGg7XG4gIHZhciBzb3VyY2U7XG4gIHZhciB0YXJnZXQ7XG4gIHZhciBzaXplT2ZTb3VyY2VJbkxjYTtcbiAgdmFyIHNpemVPZlRhcmdldEluTGNhO1xuXG4gIHZhciBhbGxFZGdlcyA9IHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0QWxsRWRnZXMoKTtcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBhbGxFZGdlcy5sZW5ndGg7IGkrKylcbiAge1xuICAgIGVkZ2UgPSBhbGxFZGdlc1tpXTtcblxuICAgIGVkZ2UuaWRlYWxMZW5ndGggPSB0aGlzLmlkZWFsRWRnZUxlbmd0aDtcblxuICAgIGlmIChlZGdlLmlzSW50ZXJHcmFwaClcbiAgICB7XG4gICAgICBzb3VyY2UgPSBlZGdlLmdldFNvdXJjZSgpO1xuICAgICAgdGFyZ2V0ID0gZWRnZS5nZXRUYXJnZXQoKTtcblxuICAgICAgc2l6ZU9mU291cmNlSW5MY2EgPSBlZGdlLmdldFNvdXJjZUluTGNhKCkuZ2V0RXN0aW1hdGVkU2l6ZSgpO1xuICAgICAgc2l6ZU9mVGFyZ2V0SW5MY2EgPSBlZGdlLmdldFRhcmdldEluTGNhKCkuZ2V0RXN0aW1hdGVkU2l6ZSgpO1xuXG4gICAgICBpZiAodGhpcy51c2VTbWFydElkZWFsRWRnZUxlbmd0aENhbGN1bGF0aW9uKVxuICAgICAge1xuICAgICAgICBlZGdlLmlkZWFsTGVuZ3RoICs9IHNpemVPZlNvdXJjZUluTGNhICsgc2l6ZU9mVGFyZ2V0SW5MY2EgLVxuICAgICAgICAgICAgICAgIDIgKiBMYXlvdXRDb25zdGFudHMuU0lNUExFX05PREVfU0laRTtcbiAgICAgIH1cblxuICAgICAgbGNhRGVwdGggPSBlZGdlLmdldExjYSgpLmdldEluY2x1c2lvblRyZWVEZXB0aCgpO1xuXG4gICAgICBlZGdlLmlkZWFsTGVuZ3RoICs9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEggKlxuICAgICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5QRVJfTEVWRUxfSURFQUxfRURHRV9MRU5HVEhfRkFDVE9SICpcbiAgICAgICAgICAgICAgKHNvdXJjZS5nZXRJbmNsdXNpb25UcmVlRGVwdGgoKSArXG4gICAgICAgICAgICAgICAgICAgICAgdGFyZ2V0LmdldEluY2x1c2lvblRyZWVEZXB0aCgpIC0gMiAqIGxjYURlcHRoKTtcbiAgICB9XG4gIH1cbn07XG5cbkZETGF5b3V0LnByb3RvdHlwZS5pbml0U3ByaW5nRW1iZWRkZXIgPSBmdW5jdGlvbiAoKSB7XG5cbiAgaWYgKHRoaXMuaW5jcmVtZW50YWwpXG4gIHtcbiAgICB0aGlzLmNvb2xpbmdGYWN0b3IgPSAwLjg7XG4gICAgdGhpcy5pbml0aWFsQ29vbGluZ0ZhY3RvciA9IDAuODtcbiAgICB0aGlzLm1heE5vZGVEaXNwbGFjZW1lbnQgPVxuICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuTUFYX05PREVfRElTUExBQ0VNRU5UX0lOQ1JFTUVOVEFMO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIHRoaXMuY29vbGluZ0ZhY3RvciA9IDEuMDtcbiAgICB0aGlzLmluaXRpYWxDb29saW5nRmFjdG9yID0gMS4wO1xuICAgIHRoaXMubWF4Tm9kZURpc3BsYWNlbWVudCA9XG4gICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5NQVhfTk9ERV9ESVNQTEFDRU1FTlQ7XG4gIH1cblxuICB0aGlzLm1heEl0ZXJhdGlvbnMgPVxuICAgICAgICAgIE1hdGgubWF4KHRoaXMuZ2V0QWxsTm9kZXMoKS5sZW5ndGggKiA1LCB0aGlzLm1heEl0ZXJhdGlvbnMpO1xuXG4gIHRoaXMudG90YWxEaXNwbGFjZW1lbnRUaHJlc2hvbGQgPVxuICAgICAgICAgIHRoaXMuZGlzcGxhY2VtZW50VGhyZXNob2xkUGVyTm9kZSAqIHRoaXMuZ2V0QWxsTm9kZXMoKS5sZW5ndGg7XG5cbiAgdGhpcy5yZXB1bHNpb25SYW5nZSA9IHRoaXMuY2FsY1JlcHVsc2lvblJhbmdlKCk7XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuY2FsY1NwcmluZ0ZvcmNlcyA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGxFZGdlcyA9IHRoaXMuZ2V0QWxsRWRnZXMoKTtcbiAgdmFyIGVkZ2U7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBsRWRnZXMubGVuZ3RoOyBpKyspXG4gIHtcbiAgICBlZGdlID0gbEVkZ2VzW2ldO1xuXG4gICAgdGhpcy5jYWxjU3ByaW5nRm9yY2UoZWRnZSwgZWRnZS5pZGVhbExlbmd0aCk7XG4gIH1cbn07XG5cbkZETGF5b3V0LnByb3RvdHlwZS5jYWxjUmVwdWxzaW9uRm9yY2VzID0gZnVuY3Rpb24gKCkge1xuICB2YXIgaSwgajtcbiAgdmFyIG5vZGVBLCBub2RlQjtcbiAgdmFyIGxOb2RlcyA9IHRoaXMuZ2V0QWxsTm9kZXMoKTtcblxuICBmb3IgKGkgPSAwOyBpIDwgbE5vZGVzLmxlbmd0aDsgaSsrKVxuICB7XG4gICAgbm9kZUEgPSBsTm9kZXNbaV07XG5cbiAgICBmb3IgKGogPSBpICsgMTsgaiA8IGxOb2Rlcy5sZW5ndGg7IGorKylcbiAgICB7XG4gICAgICBub2RlQiA9IGxOb2Rlc1tqXTtcblxuICAgICAgLy8gSWYgYm90aCBub2RlcyBhcmUgbm90IG1lbWJlcnMgb2YgdGhlIHNhbWUgZ3JhcGgsIHNraXAuXG4gICAgICBpZiAobm9kZUEuZ2V0T3duZXIoKSAhPSBub2RlQi5nZXRPd25lcigpKVxuICAgICAge1xuICAgICAgICBjb250aW51ZTtcbiAgICAgIH1cblxuICAgICAgdGhpcy5jYWxjUmVwdWxzaW9uRm9yY2Uobm9kZUEsIG5vZGVCKTtcbiAgICB9XG4gIH1cbn07XG5cbkZETGF5b3V0LnByb3RvdHlwZS5jYWxjR3Jhdml0YXRpb25hbEZvcmNlcyA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIG5vZGU7XG4gIHZhciBsTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzVG9BcHBseUdyYXZpdGF0aW9uKCk7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBsTm9kZXMubGVuZ3RoOyBpKyspXG4gIHtcbiAgICBub2RlID0gbE5vZGVzW2ldO1xuICAgIHRoaXMuY2FsY0dyYXZpdGF0aW9uYWxGb3JjZShub2RlKTtcbiAgfVxufTtcblxuRkRMYXlvdXQucHJvdG90eXBlLm1vdmVOb2RlcyA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGxOb2RlcyA9IHRoaXMuZ2V0QWxsTm9kZXMoKTtcbiAgdmFyIG5vZGU7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBsTm9kZXMubGVuZ3RoOyBpKyspXG4gIHtcbiAgICBub2RlID0gbE5vZGVzW2ldO1xuICAgIG5vZGUubW92ZSgpO1xuICB9XG59XG5cbkZETGF5b3V0LnByb3RvdHlwZS5jYWxjU3ByaW5nRm9yY2UgPSBmdW5jdGlvbiAoZWRnZSwgaWRlYWxMZW5ndGgpIHtcbiAgdmFyIHNvdXJjZU5vZGUgPSBlZGdlLmdldFNvdXJjZSgpO1xuICB2YXIgdGFyZ2V0Tm9kZSA9IGVkZ2UuZ2V0VGFyZ2V0KCk7XG5cbiAgdmFyIGxlbmd0aDtcbiAgdmFyIHNwcmluZ0ZvcmNlO1xuICB2YXIgc3ByaW5nRm9yY2VYO1xuICB2YXIgc3ByaW5nRm9yY2VZO1xuXG4gIC8vIFVwZGF0ZSBlZGdlIGxlbmd0aFxuICBpZiAodGhpcy51bmlmb3JtTGVhZk5vZGVTaXplcyAmJlxuICAgICAgICAgIHNvdXJjZU5vZGUuZ2V0Q2hpbGQoKSA9PSBudWxsICYmIHRhcmdldE5vZGUuZ2V0Q2hpbGQoKSA9PSBudWxsKVxuICB7XG4gICAgZWRnZS51cGRhdGVMZW5ndGhTaW1wbGUoKTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICBlZGdlLnVwZGF0ZUxlbmd0aCgpO1xuXG4gICAgaWYgKGVkZ2UuaXNPdmVybGFwaW5nU291cmNlQW5kVGFyZ2V0KVxuICAgIHtcbiAgICAgIHJldHVybjtcbiAgICB9XG4gIH1cblxuICBsZW5ndGggPSBlZGdlLmdldExlbmd0aCgpO1xuXG4gIC8vIENhbGN1bGF0ZSBzcHJpbmcgZm9yY2VzXG4gIHNwcmluZ0ZvcmNlID0gdGhpcy5zcHJpbmdDb25zdGFudCAqIChsZW5ndGggLSBpZGVhbExlbmd0aCk7XG5cbiAgLy8gUHJvamVjdCBmb3JjZSBvbnRvIHggYW5kIHkgYXhlc1xuICBzcHJpbmdGb3JjZVggPSBzcHJpbmdGb3JjZSAqIChlZGdlLmxlbmd0aFggLyBsZW5ndGgpO1xuICBzcHJpbmdGb3JjZVkgPSBzcHJpbmdGb3JjZSAqIChlZGdlLmxlbmd0aFkgLyBsZW5ndGgpO1xuXG4gIC8vIEFwcGx5IGZvcmNlcyBvbiB0aGUgZW5kIG5vZGVzXG4gIHNvdXJjZU5vZGUuc3ByaW5nRm9yY2VYICs9IHNwcmluZ0ZvcmNlWDtcbiAgc291cmNlTm9kZS5zcHJpbmdGb3JjZVkgKz0gc3ByaW5nRm9yY2VZO1xuICB0YXJnZXROb2RlLnNwcmluZ0ZvcmNlWCAtPSBzcHJpbmdGb3JjZVg7XG4gIHRhcmdldE5vZGUuc3ByaW5nRm9yY2VZIC09IHNwcmluZ0ZvcmNlWTtcbn07XG5cbkZETGF5b3V0LnByb3RvdHlwZS5jYWxjUmVwdWxzaW9uRm9yY2UgPSBmdW5jdGlvbiAobm9kZUEsIG5vZGVCKSB7XG4gIHZhciByZWN0QSA9IG5vZGVBLmdldFJlY3QoKTtcbiAgdmFyIHJlY3RCID0gbm9kZUIuZ2V0UmVjdCgpO1xuICB2YXIgb3ZlcmxhcEFtb3VudCA9IG5ldyBBcnJheSgyKTtcbiAgdmFyIGNsaXBQb2ludHMgPSBuZXcgQXJyYXkoNCk7XG4gIHZhciBkaXN0YW5jZVg7XG4gIHZhciBkaXN0YW5jZVk7XG4gIHZhciBkaXN0YW5jZVNxdWFyZWQ7XG4gIHZhciBkaXN0YW5jZTtcbiAgdmFyIHJlcHVsc2lvbkZvcmNlO1xuICB2YXIgcmVwdWxzaW9uRm9yY2VYO1xuICB2YXIgcmVwdWxzaW9uRm9yY2VZO1xuXG4gIGlmIChyZWN0QS5pbnRlcnNlY3RzKHJlY3RCKSkvLyB0d28gbm9kZXMgb3ZlcmxhcFxuICB7XG4gICAgLy8gY2FsY3VsYXRlIHNlcGFyYXRpb24gYW1vdW50IGluIHggYW5kIHkgZGlyZWN0aW9uc1xuICAgIElHZW9tZXRyeS5jYWxjU2VwYXJhdGlvbkFtb3VudChyZWN0QSxcbiAgICAgICAgICAgIHJlY3RCLFxuICAgICAgICAgICAgb3ZlcmxhcEFtb3VudCxcbiAgICAgICAgICAgIEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEggLyAyLjApO1xuXG4gICAgcmVwdWxzaW9uRm9yY2VYID0gb3ZlcmxhcEFtb3VudFswXTtcbiAgICByZXB1bHNpb25Gb3JjZVkgPSBvdmVybGFwQW1vdW50WzFdO1xuICB9XG4gIGVsc2UvLyBubyBvdmVybGFwXG4gIHtcbiAgICAvLyBjYWxjdWxhdGUgZGlzdGFuY2VcblxuICAgIGlmICh0aGlzLnVuaWZvcm1MZWFmTm9kZVNpemVzICYmXG4gICAgICAgICAgICBub2RlQS5nZXRDaGlsZCgpID09IG51bGwgJiYgbm9kZUIuZ2V0Q2hpbGQoKSA9PSBudWxsKS8vIHNpbXBseSBiYXNlIHJlcHVsc2lvbiBvbiBkaXN0YW5jZSBvZiBub2RlIGNlbnRlcnNcbiAgICB7XG4gICAgICBkaXN0YW5jZVggPSByZWN0Qi5nZXRDZW50ZXJYKCkgLSByZWN0QS5nZXRDZW50ZXJYKCk7XG4gICAgICBkaXN0YW5jZVkgPSByZWN0Qi5nZXRDZW50ZXJZKCkgLSByZWN0QS5nZXRDZW50ZXJZKCk7XG4gICAgfVxuICAgIGVsc2UvLyB1c2UgY2xpcHBpbmcgcG9pbnRzXG4gICAge1xuICAgICAgSUdlb21ldHJ5LmdldEludGVyc2VjdGlvbihyZWN0QSwgcmVjdEIsIGNsaXBQb2ludHMpO1xuXG4gICAgICBkaXN0YW5jZVggPSBjbGlwUG9pbnRzWzJdIC0gY2xpcFBvaW50c1swXTtcbiAgICAgIGRpc3RhbmNlWSA9IGNsaXBQb2ludHNbM10gLSBjbGlwUG9pbnRzWzFdO1xuICAgIH1cblxuICAgIC8vIE5vIHJlcHVsc2lvbiByYW5nZS4gRlIgZ3JpZCB2YXJpYW50IHNob3VsZCB0YWtlIGNhcmUgb2YgdGhpcy5cbiAgICBpZiAoTWF0aC5hYnMoZGlzdGFuY2VYKSA8IEZETGF5b3V0Q29uc3RhbnRzLk1JTl9SRVBVTFNJT05fRElTVClcbiAgICB7XG4gICAgICBkaXN0YW5jZVggPSBJTWF0aC5zaWduKGRpc3RhbmNlWCkgKlxuICAgICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5NSU5fUkVQVUxTSU9OX0RJU1Q7XG4gICAgfVxuXG4gICAgaWYgKE1hdGguYWJzKGRpc3RhbmNlWSkgPCBGRExheW91dENvbnN0YW50cy5NSU5fUkVQVUxTSU9OX0RJU1QpXG4gICAge1xuICAgICAgZGlzdGFuY2VZID0gSU1hdGguc2lnbihkaXN0YW5jZVkpICpcbiAgICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuTUlOX1JFUFVMU0lPTl9ESVNUO1xuICAgIH1cblxuICAgIGRpc3RhbmNlU3F1YXJlZCA9IGRpc3RhbmNlWCAqIGRpc3RhbmNlWCArIGRpc3RhbmNlWSAqIGRpc3RhbmNlWTtcbiAgICBkaXN0YW5jZSA9IE1hdGguc3FydChkaXN0YW5jZVNxdWFyZWQpO1xuXG4gICAgcmVwdWxzaW9uRm9yY2UgPSB0aGlzLnJlcHVsc2lvbkNvbnN0YW50IC8gZGlzdGFuY2VTcXVhcmVkO1xuXG4gICAgLy8gUHJvamVjdCBmb3JjZSBvbnRvIHggYW5kIHkgYXhlc1xuICAgIHJlcHVsc2lvbkZvcmNlWCA9IHJlcHVsc2lvbkZvcmNlICogZGlzdGFuY2VYIC8gZGlzdGFuY2U7XG4gICAgcmVwdWxzaW9uRm9yY2VZID0gcmVwdWxzaW9uRm9yY2UgKiBkaXN0YW5jZVkgLyBkaXN0YW5jZTtcbiAgfVxuXG4gIC8vIEFwcGx5IGZvcmNlcyBvbiB0aGUgdHdvIG5vZGVzXG4gIG5vZGVBLnJlcHVsc2lvbkZvcmNlWCAtPSByZXB1bHNpb25Gb3JjZVg7XG4gIG5vZGVBLnJlcHVsc2lvbkZvcmNlWSAtPSByZXB1bHNpb25Gb3JjZVk7XG4gIG5vZGVCLnJlcHVsc2lvbkZvcmNlWCArPSByZXB1bHNpb25Gb3JjZVg7XG4gIG5vZGVCLnJlcHVsc2lvbkZvcmNlWSArPSByZXB1bHNpb25Gb3JjZVk7XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuY2FsY0dyYXZpdGF0aW9uYWxGb3JjZSA9IGZ1bmN0aW9uIChub2RlKSB7XG4gIHZhciBvd25lckdyYXBoO1xuICB2YXIgb3duZXJDZW50ZXJYO1xuICB2YXIgb3duZXJDZW50ZXJZO1xuICB2YXIgZGlzdGFuY2VYO1xuICB2YXIgZGlzdGFuY2VZO1xuICB2YXIgYWJzRGlzdGFuY2VYO1xuICB2YXIgYWJzRGlzdGFuY2VZO1xuICB2YXIgZXN0aW1hdGVkU2l6ZTtcbiAgb3duZXJHcmFwaCA9IG5vZGUuZ2V0T3duZXIoKTtcblxuICBvd25lckNlbnRlclggPSAob3duZXJHcmFwaC5nZXRSaWdodCgpICsgb3duZXJHcmFwaC5nZXRMZWZ0KCkpIC8gMjtcbiAgb3duZXJDZW50ZXJZID0gKG93bmVyR3JhcGguZ2V0VG9wKCkgKyBvd25lckdyYXBoLmdldEJvdHRvbSgpKSAvIDI7XG4gIGRpc3RhbmNlWCA9IG5vZGUuZ2V0Q2VudGVyWCgpIC0gb3duZXJDZW50ZXJYO1xuICBkaXN0YW5jZVkgPSBub2RlLmdldENlbnRlclkoKSAtIG93bmVyQ2VudGVyWTtcbiAgYWJzRGlzdGFuY2VYID0gTWF0aC5hYnMoZGlzdGFuY2VYKTtcbiAgYWJzRGlzdGFuY2VZID0gTWF0aC5hYnMoZGlzdGFuY2VZKTtcblxuICBpZiAobm9kZS5nZXRPd25lcigpID09IHRoaXMuZ3JhcGhNYW5hZ2VyLmdldFJvb3QoKSkvLyBpbiB0aGUgcm9vdCBncmFwaFxuICB7XG4gICAgTWF0aC5mbG9vcig4MCk7XG4gICAgZXN0aW1hdGVkU2l6ZSA9IE1hdGguZmxvb3Iob3duZXJHcmFwaC5nZXRFc3RpbWF0ZWRTaXplKCkgKlxuICAgICAgICAgICAgdGhpcy5ncmF2aXR5UmFuZ2VGYWN0b3IpO1xuXG4gICAgaWYgKGFic0Rpc3RhbmNlWCA+IGVzdGltYXRlZFNpemUgfHwgYWJzRGlzdGFuY2VZID4gZXN0aW1hdGVkU2l6ZSlcbiAgICB7XG4gICAgICBub2RlLmdyYXZpdGF0aW9uRm9yY2VYID0gLXRoaXMuZ3Jhdml0eUNvbnN0YW50ICogZGlzdGFuY2VYO1xuICAgICAgbm9kZS5ncmF2aXRhdGlvbkZvcmNlWSA9IC10aGlzLmdyYXZpdHlDb25zdGFudCAqIGRpc3RhbmNlWTtcbiAgICB9XG4gIH1cbiAgZWxzZS8vIGluc2lkZSBhIGNvbXBvdW5kXG4gIHtcbiAgICBlc3RpbWF0ZWRTaXplID0gTWF0aC5mbG9vcigob3duZXJHcmFwaC5nZXRFc3RpbWF0ZWRTaXplKCkgKlxuICAgICAgICAgICAgdGhpcy5jb21wb3VuZEdyYXZpdHlSYW5nZUZhY3RvcikpO1xuXG4gICAgaWYgKGFic0Rpc3RhbmNlWCA+IGVzdGltYXRlZFNpemUgfHwgYWJzRGlzdGFuY2VZID4gZXN0aW1hdGVkU2l6ZSlcbiAgICB7XG4gICAgICBub2RlLmdyYXZpdGF0aW9uRm9yY2VYID0gLXRoaXMuZ3Jhdml0eUNvbnN0YW50ICogZGlzdGFuY2VYICpcbiAgICAgICAgICAgICAgdGhpcy5jb21wb3VuZEdyYXZpdHlDb25zdGFudDtcbiAgICAgIG5vZGUuZ3Jhdml0YXRpb25Gb3JjZVkgPSAtdGhpcy5ncmF2aXR5Q29uc3RhbnQgKiBkaXN0YW5jZVkgKlxuICAgICAgICAgICAgICB0aGlzLmNvbXBvdW5kR3Jhdml0eUNvbnN0YW50O1xuICAgIH1cbiAgfVxufTtcblxuRkRMYXlvdXQucHJvdG90eXBlLmlzQ29udmVyZ2VkID0gZnVuY3Rpb24gKCkge1xuICB2YXIgY29udmVyZ2VkO1xuICB2YXIgb3NjaWxhdGluZyA9IGZhbHNlO1xuXG4gIGlmICh0aGlzLnRvdGFsSXRlcmF0aW9ucyA+IHRoaXMubWF4SXRlcmF0aW9ucyAvIDMpXG4gIHtcbiAgICBvc2NpbGF0aW5nID1cbiAgICAgICAgICAgIE1hdGguYWJzKHRoaXMudG90YWxEaXNwbGFjZW1lbnQgLSB0aGlzLm9sZFRvdGFsRGlzcGxhY2VtZW50KSA8IDI7XG4gIH1cblxuICBjb252ZXJnZWQgPSB0aGlzLnRvdGFsRGlzcGxhY2VtZW50IDwgdGhpcy50b3RhbERpc3BsYWNlbWVudFRocmVzaG9sZDtcblxuICB0aGlzLm9sZFRvdGFsRGlzcGxhY2VtZW50ID0gdGhpcy50b3RhbERpc3BsYWNlbWVudDtcblxuICByZXR1cm4gY29udmVyZ2VkIHx8IG9zY2lsYXRpbmc7XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuYW5pbWF0ZSA9IGZ1bmN0aW9uICgpIHtcbiAgaWYgKHRoaXMuYW5pbWF0aW9uRHVyaW5nTGF5b3V0ICYmICF0aGlzLmlzU3ViTGF5b3V0KVxuICB7XG4gICAgaWYgKHRoaXMubm90QW5pbWF0ZWRJdGVyYXRpb25zID09IHRoaXMuYW5pbWF0aW9uUGVyaW9kKVxuICAgIHtcbiAgICAgIHRoaXMudXBkYXRlKCk7XG4gICAgICB0aGlzLm5vdEFuaW1hdGVkSXRlcmF0aW9ucyA9IDA7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICB0aGlzLm5vdEFuaW1hdGVkSXRlcmF0aW9ucysrO1xuICAgIH1cbiAgfVxufTtcblxuRkRMYXlvdXQucHJvdG90eXBlLmNhbGNSZXB1bHNpb25SYW5nZSA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIDAuMDtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gRkRMYXlvdXQ7XG4iLCJ2YXIgTGF5b3V0Q29uc3RhbnRzID0gcmVxdWlyZSgnLi9MYXlvdXRDb25zdGFudHMnKTtcblxuZnVuY3Rpb24gRkRMYXlvdXRDb25zdGFudHMoKSB7XG59XG5cbi8vRkRMYXlvdXRDb25zdGFudHMgaW5oZXJpdHMgc3RhdGljIHByb3BzIGluIExheW91dENvbnN0YW50c1xuZm9yICh2YXIgcHJvcCBpbiBMYXlvdXRDb25zdGFudHMpIHtcbiAgRkRMYXlvdXRDb25zdGFudHNbcHJvcF0gPSBMYXlvdXRDb25zdGFudHNbcHJvcF07XG59XG5cbkZETGF5b3V0Q29uc3RhbnRzLk1BWF9JVEVSQVRJT05TID0gMjUwMDtcblxuRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9FREdFX0xFTkdUSCA9IDUwO1xuRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9TUFJJTkdfU1RSRU5HVEggPSAwLjQ1O1xuRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9SRVBVTFNJT05fU1RSRU5HVEggPSA0NTAwLjA7XG5GRExheW91dENvbnN0YW50cy5ERUZBVUxUX0dSQVZJVFlfU1RSRU5HVEggPSAwLjQ7XG5GRExheW91dENvbnN0YW50cy5ERUZBVUxUX0NPTVBPVU5EX0dSQVZJVFlfU1RSRU5HVEggPSAxLjA7XG5GRExheW91dENvbnN0YW50cy5ERUZBVUxUX0dSQVZJVFlfUkFOR0VfRkFDVE9SID0gMy44O1xuRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IDEuNTtcbkZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfVVNFX1NNQVJUX0lERUFMX0VER0VfTEVOR1RIX0NBTENVTEFUSU9OID0gdHJ1ZTtcbkZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfVVNFX1NNQVJUX1JFUFVMU0lPTl9SQU5HRV9DQUxDVUxBVElPTiA9IHRydWU7XG5GRExheW91dENvbnN0YW50cy5NQVhfTk9ERV9ESVNQTEFDRU1FTlRfSU5DUkVNRU5UQUwgPSAxMDAuMDtcbkZETGF5b3V0Q29uc3RhbnRzLk1BWF9OT0RFX0RJU1BMQUNFTUVOVCA9IEZETGF5b3V0Q29uc3RhbnRzLk1BWF9OT0RFX0RJU1BMQUNFTUVOVF9JTkNSRU1FTlRBTCAqIDM7XG5GRExheW91dENvbnN0YW50cy5NSU5fUkVQVUxTSU9OX0RJU1QgPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIIC8gMTAuMDtcbkZETGF5b3V0Q29uc3RhbnRzLkNPTlZFUkdFTkNFX0NIRUNLX1BFUklPRCA9IDEwMDtcbkZETGF5b3V0Q29uc3RhbnRzLlBFUl9MRVZFTF9JREVBTF9FREdFX0xFTkdUSF9GQUNUT1IgPSAwLjE7XG5GRExheW91dENvbnN0YW50cy5NSU5fRURHRV9MRU5HVEggPSAxO1xuRkRMYXlvdXRDb25zdGFudHMuR1JJRF9DQUxDVUxBVElPTl9DSEVDS19QRVJJT0QgPSAxMDtcblxubW9kdWxlLmV4cG9ydHMgPSBGRExheW91dENvbnN0YW50cztcbiIsInZhciBMRWRnZSA9IHJlcXVpcmUoJy4vTEVkZ2UnKTtcbnZhciBGRExheW91dENvbnN0YW50cyA9IHJlcXVpcmUoJy4vRkRMYXlvdXRDb25zdGFudHMnKTtcblxuZnVuY3Rpb24gRkRMYXlvdXRFZGdlKHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSkge1xuICBMRWRnZS5jYWxsKHRoaXMsIHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSk7XG4gIHRoaXMuaWRlYWxMZW5ndGggPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIO1xufVxuXG5GRExheW91dEVkZ2UucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMRWRnZS5wcm90b3R5cGUpO1xuXG5mb3IgKHZhciBwcm9wIGluIExFZGdlKSB7XG4gIEZETGF5b3V0RWRnZVtwcm9wXSA9IExFZGdlW3Byb3BdO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IEZETGF5b3V0RWRnZTtcbiIsInZhciBMTm9kZSA9IHJlcXVpcmUoJy4vTE5vZGUnKTtcblxuZnVuY3Rpb24gRkRMYXlvdXROb2RlKGdtLCBsb2MsIHNpemUsIHZOb2RlKSB7XG4gIC8vIGFsdGVybmF0aXZlIGNvbnN0cnVjdG9yIGlzIGhhbmRsZWQgaW5zaWRlIExOb2RlXG4gIExOb2RlLmNhbGwodGhpcywgZ20sIGxvYywgc2l6ZSwgdk5vZGUpO1xuICAvL1NwcmluZywgcmVwdWxzaW9uIGFuZCBncmF2aXRhdGlvbmFsIGZvcmNlcyBhY3Rpbmcgb24gdGhpcyBub2RlXG4gIHRoaXMuc3ByaW5nRm9yY2VYID0gMDtcbiAgdGhpcy5zcHJpbmdGb3JjZVkgPSAwO1xuICB0aGlzLnJlcHVsc2lvbkZvcmNlWCA9IDA7XG4gIHRoaXMucmVwdWxzaW9uRm9yY2VZID0gMDtcbiAgdGhpcy5ncmF2aXRhdGlvbkZvcmNlWCA9IDA7XG4gIHRoaXMuZ3Jhdml0YXRpb25Gb3JjZVkgPSAwO1xuICAvL0Ftb3VudCBieSB3aGljaCB0aGlzIG5vZGUgaXMgdG8gYmUgbW92ZWQgaW4gdGhpcyBpdGVyYXRpb25cbiAgdGhpcy5kaXNwbGFjZW1lbnRYID0gMDtcbiAgdGhpcy5kaXNwbGFjZW1lbnRZID0gMDtcblxuICAvL1N0YXJ0IGFuZCBmaW5pc2ggZ3JpZCBjb29yZGluYXRlcyB0aGF0IHRoaXMgbm9kZSBpcyBmYWxsZW4gaW50b1xuICB0aGlzLnN0YXJ0WCA9IDA7XG4gIHRoaXMuZmluaXNoWCA9IDA7XG4gIHRoaXMuc3RhcnRZID0gMDtcbiAgdGhpcy5maW5pc2hZID0gMDtcblxuICAvL0dlb21ldHJpYyBuZWlnaGJvcnMgb2YgdGhpcyBub2RlXG4gIHRoaXMuc3Vycm91bmRpbmcgPSBbXTtcbn1cblxuRkRMYXlvdXROb2RlLnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoTE5vZGUucHJvdG90eXBlKTtcblxuZm9yICh2YXIgcHJvcCBpbiBMTm9kZSkge1xuICBGRExheW91dE5vZGVbcHJvcF0gPSBMTm9kZVtwcm9wXTtcbn1cblxuRkRMYXlvdXROb2RlLnByb3RvdHlwZS5zZXRHcmlkQ29vcmRpbmF0ZXMgPSBmdW5jdGlvbiAoX3N0YXJ0WCwgX2ZpbmlzaFgsIF9zdGFydFksIF9maW5pc2hZKVxue1xuICB0aGlzLnN0YXJ0WCA9IF9zdGFydFg7XG4gIHRoaXMuZmluaXNoWCA9IF9maW5pc2hYO1xuICB0aGlzLnN0YXJ0WSA9IF9zdGFydFk7XG4gIHRoaXMuZmluaXNoWSA9IF9maW5pc2hZO1xuXG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IEZETGF5b3V0Tm9kZTtcbiIsInZhciBVbmlxdWVJREdlbmVyZXRvciA9IHJlcXVpcmUoJy4vVW5pcXVlSURHZW5lcmV0b3InKTtcblxuZnVuY3Rpb24gSGFzaE1hcCgpIHtcbiAgdGhpcy5tYXAgPSB7fTtcbiAgdGhpcy5rZXlzID0gW107XG59XG5cbkhhc2hNYXAucHJvdG90eXBlLnB1dCA9IGZ1bmN0aW9uIChrZXksIHZhbHVlKSB7XG4gIHZhciB0aGVJZCA9IFVuaXF1ZUlER2VuZXJldG9yLmNyZWF0ZUlEKGtleSk7XG4gIGlmICghdGhpcy5jb250YWlucyh0aGVJZCkpIHtcbiAgICB0aGlzLm1hcFt0aGVJZF0gPSB2YWx1ZTtcbiAgICB0aGlzLmtleXMucHVzaChrZXkpO1xuICB9XG59O1xuXG5IYXNoTWFwLnByb3RvdHlwZS5jb250YWlucyA9IGZ1bmN0aW9uIChrZXkpIHtcbiAgdmFyIHRoZUlkID0gVW5pcXVlSURHZW5lcmV0b3IuY3JlYXRlSUQoa2V5KTtcbiAgcmV0dXJuIHRoaXMubWFwW2tleV0gIT0gbnVsbDtcbn07XG5cbkhhc2hNYXAucHJvdG90eXBlLmdldCA9IGZ1bmN0aW9uIChrZXkpIHtcbiAgdmFyIHRoZUlkID0gVW5pcXVlSURHZW5lcmV0b3IuY3JlYXRlSUQoa2V5KTtcbiAgcmV0dXJuIHRoaXMubWFwW3RoZUlkXTtcbn07XG5cbkhhc2hNYXAucHJvdG90eXBlLmtleVNldCA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIHRoaXMua2V5cztcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gSGFzaE1hcDtcbiIsInZhciBVbmlxdWVJREdlbmVyZXRvciA9IHJlcXVpcmUoJy4vVW5pcXVlSURHZW5lcmV0b3InKTtcblxuZnVuY3Rpb24gSGFzaFNldCgpIHtcbiAgdGhpcy5zZXQgPSB7fTtcbn1cbjtcblxuSGFzaFNldC5wcm90b3R5cGUuYWRkID0gZnVuY3Rpb24gKG9iaikge1xuICB2YXIgdGhlSWQgPSBVbmlxdWVJREdlbmVyZXRvci5jcmVhdGVJRChvYmopO1xuICBpZiAoIXRoaXMuY29udGFpbnModGhlSWQpKVxuICAgIHRoaXMuc2V0W3RoZUlkXSA9IG9iajtcbn07XG5cbkhhc2hTZXQucHJvdG90eXBlLnJlbW92ZSA9IGZ1bmN0aW9uIChvYmopIHtcbiAgZGVsZXRlIHRoaXMuc2V0W1VuaXF1ZUlER2VuZXJldG9yLmNyZWF0ZUlEKG9iaildO1xufTtcblxuSGFzaFNldC5wcm90b3R5cGUuY2xlYXIgPSBmdW5jdGlvbiAoKSB7XG4gIHRoaXMuc2V0ID0ge307XG59O1xuXG5IYXNoU2V0LnByb3RvdHlwZS5jb250YWlucyA9IGZ1bmN0aW9uIChvYmopIHtcbiAgcmV0dXJuIHRoaXMuc2V0W1VuaXF1ZUlER2VuZXJldG9yLmNyZWF0ZUlEKG9iaildID09IG9iajtcbn07XG5cbkhhc2hTZXQucHJvdG90eXBlLmlzRW1wdHkgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiB0aGlzLnNpemUoKSA9PT0gMDtcbn07XG5cbkhhc2hTZXQucHJvdG90eXBlLnNpemUgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiBPYmplY3Qua2V5cyh0aGlzLnNldCkubGVuZ3RoO1xufTtcblxuLy9jb25jYXRzIHRoaXMuc2V0IHRvIHRoZSBnaXZlbiBsaXN0XG5IYXNoU2V0LnByb3RvdHlwZS5hZGRBbGxUbyA9IGZ1bmN0aW9uIChsaXN0KSB7XG4gIHZhciBrZXlzID0gT2JqZWN0LmtleXModGhpcy5zZXQpO1xuICB2YXIgbGVuZ3RoID0ga2V5cy5sZW5ndGg7XG4gIGZvciAodmFyIGkgPSAwOyBpIDwgbGVuZ3RoOyBpKyspIHtcbiAgICBsaXN0LnB1c2godGhpcy5zZXRba2V5c1tpXV0pO1xuICB9XG59O1xuXG5IYXNoU2V0LnByb3RvdHlwZS5zaXplID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gT2JqZWN0LmtleXModGhpcy5zZXQpLmxlbmd0aDtcbn07XG5cbkhhc2hTZXQucHJvdG90eXBlLmFkZEFsbCA9IGZ1bmN0aW9uIChsaXN0KSB7XG4gIHZhciBzID0gbGlzdC5sZW5ndGg7XG4gIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKSB7XG4gICAgdmFyIHYgPSBsaXN0W2ldO1xuICAgIHRoaXMuYWRkKHYpO1xuICB9XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IEhhc2hTZXQ7XG4iLCJmdW5jdGlvbiBJR2VvbWV0cnkoKSB7XG59XG5cbklHZW9tZXRyeS5jYWxjU2VwYXJhdGlvbkFtb3VudCA9IGZ1bmN0aW9uIChyZWN0QSwgcmVjdEIsIG92ZXJsYXBBbW91bnQsIHNlcGFyYXRpb25CdWZmZXIpXG57XG4gIGlmICghcmVjdEEuaW50ZXJzZWN0cyhyZWN0QikpIHtcbiAgICB0aHJvdyBcImFzc2VydCBmYWlsZWRcIjtcbiAgfVxuICB2YXIgZGlyZWN0aW9ucyA9IG5ldyBBcnJheSgyKTtcbiAgSUdlb21ldHJ5LmRlY2lkZURpcmVjdGlvbnNGb3JPdmVybGFwcGluZ05vZGVzKHJlY3RBLCByZWN0QiwgZGlyZWN0aW9ucyk7XG4gIG92ZXJsYXBBbW91bnRbMF0gPSBNYXRoLm1pbihyZWN0QS5nZXRSaWdodCgpLCByZWN0Qi5nZXRSaWdodCgpKSAtXG4gICAgICAgICAgTWF0aC5tYXgocmVjdEEueCwgcmVjdEIueCk7XG4gIG92ZXJsYXBBbW91bnRbMV0gPSBNYXRoLm1pbihyZWN0QS5nZXRCb3R0b20oKSwgcmVjdEIuZ2V0Qm90dG9tKCkpIC1cbiAgICAgICAgICBNYXRoLm1heChyZWN0QS55LCByZWN0Qi55KTtcbiAgLy8gdXBkYXRlIHRoZSBvdmVybGFwcGluZyBhbW91bnRzIGZvciB0aGUgZm9sbG93aW5nIGNhc2VzOlxuICBpZiAoKHJlY3RBLmdldFgoKSA8PSByZWN0Qi5nZXRYKCkpICYmIChyZWN0QS5nZXRSaWdodCgpID49IHJlY3RCLmdldFJpZ2h0KCkpKVxuICB7XG4gICAgb3ZlcmxhcEFtb3VudFswXSArPSBNYXRoLm1pbigocmVjdEIuZ2V0WCgpIC0gcmVjdEEuZ2V0WCgpKSxcbiAgICAgICAgICAgIChyZWN0QS5nZXRSaWdodCgpIC0gcmVjdEIuZ2V0UmlnaHQoKSkpO1xuICB9XG4gIGVsc2UgaWYgKChyZWN0Qi5nZXRYKCkgPD0gcmVjdEEuZ2V0WCgpKSAmJiAocmVjdEIuZ2V0UmlnaHQoKSA+PSByZWN0QS5nZXRSaWdodCgpKSlcbiAge1xuICAgIG92ZXJsYXBBbW91bnRbMF0gKz0gTWF0aC5taW4oKHJlY3RBLmdldFgoKSAtIHJlY3RCLmdldFgoKSksXG4gICAgICAgICAgICAocmVjdEIuZ2V0UmlnaHQoKSAtIHJlY3RBLmdldFJpZ2h0KCkpKTtcbiAgfVxuICBpZiAoKHJlY3RBLmdldFkoKSA8PSByZWN0Qi5nZXRZKCkpICYmIChyZWN0QS5nZXRCb3R0b20oKSA+PSByZWN0Qi5nZXRCb3R0b20oKSkpXG4gIHtcbiAgICBvdmVybGFwQW1vdW50WzFdICs9IE1hdGgubWluKChyZWN0Qi5nZXRZKCkgLSByZWN0QS5nZXRZKCkpLFxuICAgICAgICAgICAgKHJlY3RBLmdldEJvdHRvbSgpIC0gcmVjdEIuZ2V0Qm90dG9tKCkpKTtcbiAgfVxuICBlbHNlIGlmICgocmVjdEIuZ2V0WSgpIDw9IHJlY3RBLmdldFkoKSkgJiYgKHJlY3RCLmdldEJvdHRvbSgpID49IHJlY3RBLmdldEJvdHRvbSgpKSlcbiAge1xuICAgIG92ZXJsYXBBbW91bnRbMV0gKz0gTWF0aC5taW4oKHJlY3RBLmdldFkoKSAtIHJlY3RCLmdldFkoKSksXG4gICAgICAgICAgICAocmVjdEIuZ2V0Qm90dG9tKCkgLSByZWN0QS5nZXRCb3R0b20oKSkpO1xuICB9XG5cbiAgLy8gZmluZCBzbG9wZSBvZiB0aGUgbGluZSBwYXNzZXMgdHdvIGNlbnRlcnNcbiAgdmFyIHNsb3BlID0gTWF0aC5hYnMoKHJlY3RCLmdldENlbnRlclkoKSAtIHJlY3RBLmdldENlbnRlclkoKSkgL1xuICAgICAgICAgIChyZWN0Qi5nZXRDZW50ZXJYKCkgLSByZWN0QS5nZXRDZW50ZXJYKCkpKTtcbiAgLy8gaWYgY2VudGVycyBhcmUgb3ZlcmxhcHBlZFxuICBpZiAoKHJlY3RCLmdldENlbnRlclkoKSA9PSByZWN0QS5nZXRDZW50ZXJZKCkpICYmXG4gICAgICAgICAgKHJlY3RCLmdldENlbnRlclgoKSA9PSByZWN0QS5nZXRDZW50ZXJYKCkpKVxuICB7XG4gICAgLy8gYXNzdW1lIHRoZSBzbG9wZSBpcyAxICg0NSBkZWdyZWUpXG4gICAgc2xvcGUgPSAxLjA7XG4gIH1cblxuICB2YXIgbW92ZUJ5WSA9IHNsb3BlICogb3ZlcmxhcEFtb3VudFswXTtcbiAgdmFyIG1vdmVCeVggPSBvdmVybGFwQW1vdW50WzFdIC8gc2xvcGU7XG4gIGlmIChvdmVybGFwQW1vdW50WzBdIDwgbW92ZUJ5WClcbiAge1xuICAgIG1vdmVCeVggPSBvdmVybGFwQW1vdW50WzBdO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIG1vdmVCeVkgPSBvdmVybGFwQW1vdW50WzFdO1xuICB9XG4gIC8vIHJldHVybiBoYWxmIHRoZSBhbW91bnQgc28gdGhhdCBpZiBlYWNoIHJlY3RhbmdsZSBpcyBtb3ZlZCBieSB0aGVzZVxuICAvLyBhbW91bnRzIGluIG9wcG9zaXRlIGRpcmVjdGlvbnMsIG92ZXJsYXAgd2lsbCBiZSByZXNvbHZlZFxuICBvdmVybGFwQW1vdW50WzBdID0gLTEgKiBkaXJlY3Rpb25zWzBdICogKChtb3ZlQnlYIC8gMikgKyBzZXBhcmF0aW9uQnVmZmVyKTtcbiAgb3ZlcmxhcEFtb3VudFsxXSA9IC0xICogZGlyZWN0aW9uc1sxXSAqICgobW92ZUJ5WSAvIDIpICsgc2VwYXJhdGlvbkJ1ZmZlcik7XG59XG5cbklHZW9tZXRyeS5kZWNpZGVEaXJlY3Rpb25zRm9yT3ZlcmxhcHBpbmdOb2RlcyA9IGZ1bmN0aW9uIChyZWN0QSwgcmVjdEIsIGRpcmVjdGlvbnMpXG57XG4gIGlmIChyZWN0QS5nZXRDZW50ZXJYKCkgPCByZWN0Qi5nZXRDZW50ZXJYKCkpXG4gIHtcbiAgICBkaXJlY3Rpb25zWzBdID0gLTE7XG4gIH1cbiAgZWxzZVxuICB7XG4gICAgZGlyZWN0aW9uc1swXSA9IDE7XG4gIH1cblxuICBpZiAocmVjdEEuZ2V0Q2VudGVyWSgpIDwgcmVjdEIuZ2V0Q2VudGVyWSgpKVxuICB7XG4gICAgZGlyZWN0aW9uc1sxXSA9IC0xO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIGRpcmVjdGlvbnNbMV0gPSAxO1xuICB9XG59XG5cbklHZW9tZXRyeS5nZXRJbnRlcnNlY3Rpb24yID0gZnVuY3Rpb24gKHJlY3RBLCByZWN0QiwgcmVzdWx0KVxue1xuICAvL3Jlc3VsdFswLTFdIHdpbGwgY29udGFpbiBjbGlwUG9pbnQgb2YgcmVjdEEsIHJlc3VsdFsyLTNdIHdpbGwgY29udGFpbiBjbGlwUG9pbnQgb2YgcmVjdEJcbiAgdmFyIHAxeCA9IHJlY3RBLmdldENlbnRlclgoKTtcbiAgdmFyIHAxeSA9IHJlY3RBLmdldENlbnRlclkoKTtcbiAgdmFyIHAyeCA9IHJlY3RCLmdldENlbnRlclgoKTtcbiAgdmFyIHAyeSA9IHJlY3RCLmdldENlbnRlclkoKTtcblxuICAvL2lmIHR3byByZWN0YW5nbGVzIGludGVyc2VjdCwgdGhlbiBjbGlwcGluZyBwb2ludHMgYXJlIGNlbnRlcnNcbiAgaWYgKHJlY3RBLmludGVyc2VjdHMocmVjdEIpKVxuICB7XG4gICAgcmVzdWx0WzBdID0gcDF4O1xuICAgIHJlc3VsdFsxXSA9IHAxeTtcbiAgICByZXN1bHRbMl0gPSBwMng7XG4gICAgcmVzdWx0WzNdID0gcDJ5O1xuICAgIHJldHVybiB0cnVlO1xuICB9XG4gIC8vdmFyaWFibGVzIGZvciByZWN0QVxuICB2YXIgdG9wTGVmdEF4ID0gcmVjdEEuZ2V0WCgpO1xuICB2YXIgdG9wTGVmdEF5ID0gcmVjdEEuZ2V0WSgpO1xuICB2YXIgdG9wUmlnaHRBeCA9IHJlY3RBLmdldFJpZ2h0KCk7XG4gIHZhciBib3R0b21MZWZ0QXggPSByZWN0QS5nZXRYKCk7XG4gIHZhciBib3R0b21MZWZ0QXkgPSByZWN0QS5nZXRCb3R0b20oKTtcbiAgdmFyIGJvdHRvbVJpZ2h0QXggPSByZWN0QS5nZXRSaWdodCgpO1xuICB2YXIgaGFsZldpZHRoQSA9IHJlY3RBLmdldFdpZHRoSGFsZigpO1xuICB2YXIgaGFsZkhlaWdodEEgPSByZWN0QS5nZXRIZWlnaHRIYWxmKCk7XG4gIC8vdmFyaWFibGVzIGZvciByZWN0QlxuICB2YXIgdG9wTGVmdEJ4ID0gcmVjdEIuZ2V0WCgpO1xuICB2YXIgdG9wTGVmdEJ5ID0gcmVjdEIuZ2V0WSgpO1xuICB2YXIgdG9wUmlnaHRCeCA9IHJlY3RCLmdldFJpZ2h0KCk7XG4gIHZhciBib3R0b21MZWZ0QnggPSByZWN0Qi5nZXRYKCk7XG4gIHZhciBib3R0b21MZWZ0QnkgPSByZWN0Qi5nZXRCb3R0b20oKTtcbiAgdmFyIGJvdHRvbVJpZ2h0QnggPSByZWN0Qi5nZXRSaWdodCgpO1xuICB2YXIgaGFsZldpZHRoQiA9IHJlY3RCLmdldFdpZHRoSGFsZigpO1xuICB2YXIgaGFsZkhlaWdodEIgPSByZWN0Qi5nZXRIZWlnaHRIYWxmKCk7XG4gIC8vZmxhZyB3aGV0aGVyIGNsaXBwaW5nIHBvaW50cyBhcmUgZm91bmRcbiAgdmFyIGNsaXBQb2ludEFGb3VuZCA9IGZhbHNlO1xuICB2YXIgY2xpcFBvaW50QkZvdW5kID0gZmFsc2U7XG5cbiAgLy8gbGluZSBpcyB2ZXJ0aWNhbFxuICBpZiAocDF4ID09IHAyeClcbiAge1xuICAgIGlmIChwMXkgPiBwMnkpXG4gICAge1xuICAgICAgcmVzdWx0WzBdID0gcDF4O1xuICAgICAgcmVzdWx0WzFdID0gdG9wTGVmdEF5O1xuICAgICAgcmVzdWx0WzJdID0gcDJ4O1xuICAgICAgcmVzdWx0WzNdID0gYm90dG9tTGVmdEJ5O1xuICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH1cbiAgICBlbHNlIGlmIChwMXkgPCBwMnkpXG4gICAge1xuICAgICAgcmVzdWx0WzBdID0gcDF4O1xuICAgICAgcmVzdWx0WzFdID0gYm90dG9tTGVmdEF5O1xuICAgICAgcmVzdWx0WzJdID0gcDJ4O1xuICAgICAgcmVzdWx0WzNdID0gdG9wTGVmdEJ5O1xuICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH1cbiAgICBlbHNlXG4gICAge1xuICAgICAgLy9ub3QgbGluZSwgcmV0dXJuIG51bGw7XG4gICAgfVxuICB9XG4gIC8vIGxpbmUgaXMgaG9yaXpvbnRhbFxuICBlbHNlIGlmIChwMXkgPT0gcDJ5KVxuICB7XG4gICAgaWYgKHAxeCA+IHAyeClcbiAgICB7XG4gICAgICByZXN1bHRbMF0gPSB0b3BMZWZ0QXg7XG4gICAgICByZXN1bHRbMV0gPSBwMXk7XG4gICAgICByZXN1bHRbMl0gPSB0b3BSaWdodEJ4O1xuICAgICAgcmVzdWx0WzNdID0gcDJ5O1xuICAgICAgcmV0dXJuIGZhbHNlO1xuICAgIH1cbiAgICBlbHNlIGlmIChwMXggPCBwMngpXG4gICAge1xuICAgICAgcmVzdWx0WzBdID0gdG9wUmlnaHRBeDtcbiAgICAgIHJlc3VsdFsxXSA9IHAxeTtcbiAgICAgIHJlc3VsdFsyXSA9IHRvcExlZnRCeDtcbiAgICAgIHJlc3VsdFszXSA9IHAyeTtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG4gICAgZWxzZVxuICAgIHtcbiAgICAgIC8vbm90IHZhbGlkIGxpbmUsIHJldHVybiBudWxsO1xuICAgIH1cbiAgfVxuICBlbHNlXG4gIHtcbiAgICAvL3Nsb3BlcyBvZiByZWN0QSdzIGFuZCByZWN0QidzIGRpYWdvbmFsc1xuICAgIHZhciBzbG9wZUEgPSByZWN0QS5oZWlnaHQgLyByZWN0QS53aWR0aDtcbiAgICB2YXIgc2xvcGVCID0gcmVjdEIuaGVpZ2h0IC8gcmVjdEIud2lkdGg7XG5cbiAgICAvL3Nsb3BlIG9mIGxpbmUgYmV0d2VlbiBjZW50ZXIgb2YgcmVjdEEgYW5kIGNlbnRlciBvZiByZWN0QlxuICAgIHZhciBzbG9wZVByaW1lID0gKHAyeSAtIHAxeSkgLyAocDJ4IC0gcDF4KTtcbiAgICB2YXIgY2FyZGluYWxEaXJlY3Rpb25BO1xuICAgIHZhciBjYXJkaW5hbERpcmVjdGlvbkI7XG4gICAgdmFyIHRlbXBQb2ludEF4O1xuICAgIHZhciB0ZW1wUG9pbnRBeTtcbiAgICB2YXIgdGVtcFBvaW50Qng7XG4gICAgdmFyIHRlbXBQb2ludEJ5O1xuXG4gICAgLy9kZXRlcm1pbmUgd2hldGhlciBjbGlwcGluZyBwb2ludCBpcyB0aGUgY29ybmVyIG9mIG5vZGVBXG4gICAgaWYgKCgtc2xvcGVBKSA9PSBzbG9wZVByaW1lKVxuICAgIHtcbiAgICAgIGlmIChwMXggPiBwMngpXG4gICAgICB7XG4gICAgICAgIHJlc3VsdFswXSA9IGJvdHRvbUxlZnRBeDtcbiAgICAgICAgcmVzdWx0WzFdID0gYm90dG9tTGVmdEF5O1xuICAgICAgICBjbGlwUG9pbnRBRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgICAgZWxzZVxuICAgICAge1xuICAgICAgICByZXN1bHRbMF0gPSB0b3BSaWdodEF4O1xuICAgICAgICByZXN1bHRbMV0gPSB0b3BMZWZ0QXk7XG4gICAgICAgIGNsaXBQb2ludEFGb3VuZCA9IHRydWU7XG4gICAgICB9XG4gICAgfVxuICAgIGVsc2UgaWYgKHNsb3BlQSA9PSBzbG9wZVByaW1lKVxuICAgIHtcbiAgICAgIGlmIChwMXggPiBwMngpXG4gICAgICB7XG4gICAgICAgIHJlc3VsdFswXSA9IHRvcExlZnRBeDtcbiAgICAgICAgcmVzdWx0WzFdID0gdG9wTGVmdEF5O1xuICAgICAgICBjbGlwUG9pbnRBRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgICAgZWxzZVxuICAgICAge1xuICAgICAgICByZXN1bHRbMF0gPSBib3R0b21SaWdodEF4O1xuICAgICAgICByZXN1bHRbMV0gPSBib3R0b21MZWZ0QXk7XG4gICAgICAgIGNsaXBQb2ludEFGb3VuZCA9IHRydWU7XG4gICAgICB9XG4gICAgfVxuXG4gICAgLy9kZXRlcm1pbmUgd2hldGhlciBjbGlwcGluZyBwb2ludCBpcyB0aGUgY29ybmVyIG9mIG5vZGVCXG4gICAgaWYgKCgtc2xvcGVCKSA9PSBzbG9wZVByaW1lKVxuICAgIHtcbiAgICAgIGlmIChwMnggPiBwMXgpXG4gICAgICB7XG4gICAgICAgIHJlc3VsdFsyXSA9IGJvdHRvbUxlZnRCeDtcbiAgICAgICAgcmVzdWx0WzNdID0gYm90dG9tTGVmdEJ5O1xuICAgICAgICBjbGlwUG9pbnRCRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgICAgZWxzZVxuICAgICAge1xuICAgICAgICByZXN1bHRbMl0gPSB0b3BSaWdodEJ4O1xuICAgICAgICByZXN1bHRbM10gPSB0b3BMZWZ0Qnk7XG4gICAgICAgIGNsaXBQb2ludEJGb3VuZCA9IHRydWU7XG4gICAgICB9XG4gICAgfVxuICAgIGVsc2UgaWYgKHNsb3BlQiA9PSBzbG9wZVByaW1lKVxuICAgIHtcbiAgICAgIGlmIChwMnggPiBwMXgpXG4gICAgICB7XG4gICAgICAgIHJlc3VsdFsyXSA9IHRvcExlZnRCeDtcbiAgICAgICAgcmVzdWx0WzNdID0gdG9wTGVmdEJ5O1xuICAgICAgICBjbGlwUG9pbnRCRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgICAgZWxzZVxuICAgICAge1xuICAgICAgICByZXN1bHRbMl0gPSBib3R0b21SaWdodEJ4O1xuICAgICAgICByZXN1bHRbM10gPSBib3R0b21MZWZ0Qnk7XG4gICAgICAgIGNsaXBQb2ludEJGb3VuZCA9IHRydWU7XG4gICAgICB9XG4gICAgfVxuXG4gICAgLy9pZiBib3RoIGNsaXBwaW5nIHBvaW50cyBhcmUgY29ybmVyc1xuICAgIGlmIChjbGlwUG9pbnRBRm91bmQgJiYgY2xpcFBvaW50QkZvdW5kKVxuICAgIHtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG5cbiAgICAvL2RldGVybWluZSBDYXJkaW5hbCBEaXJlY3Rpb24gb2YgcmVjdGFuZ2xlc1xuICAgIGlmIChwMXggPiBwMngpXG4gICAge1xuICAgICAgaWYgKHAxeSA+IHAyeSlcbiAgICAgIHtcbiAgICAgICAgY2FyZGluYWxEaXJlY3Rpb25BID0gSUdlb21ldHJ5LmdldENhcmRpbmFsRGlyZWN0aW9uKHNsb3BlQSwgc2xvcGVQcmltZSwgNCk7XG4gICAgICAgIGNhcmRpbmFsRGlyZWN0aW9uQiA9IElHZW9tZXRyeS5nZXRDYXJkaW5hbERpcmVjdGlvbihzbG9wZUIsIHNsb3BlUHJpbWUsIDIpO1xuICAgICAgfVxuICAgICAgZWxzZVxuICAgICAge1xuICAgICAgICBjYXJkaW5hbERpcmVjdGlvbkEgPSBJR2VvbWV0cnkuZ2V0Q2FyZGluYWxEaXJlY3Rpb24oLXNsb3BlQSwgc2xvcGVQcmltZSwgMyk7XG4gICAgICAgIGNhcmRpbmFsRGlyZWN0aW9uQiA9IElHZW9tZXRyeS5nZXRDYXJkaW5hbERpcmVjdGlvbigtc2xvcGVCLCBzbG9wZVByaW1lLCAxKTtcbiAgICAgIH1cbiAgICB9XG4gICAgZWxzZVxuICAgIHtcbiAgICAgIGlmIChwMXkgPiBwMnkpXG4gICAgICB7XG4gICAgICAgIGNhcmRpbmFsRGlyZWN0aW9uQSA9IElHZW9tZXRyeS5nZXRDYXJkaW5hbERpcmVjdGlvbigtc2xvcGVBLCBzbG9wZVByaW1lLCAxKTtcbiAgICAgICAgY2FyZGluYWxEaXJlY3Rpb25CID0gSUdlb21ldHJ5LmdldENhcmRpbmFsRGlyZWN0aW9uKC1zbG9wZUIsIHNsb3BlUHJpbWUsIDMpO1xuICAgICAgfVxuICAgICAgZWxzZVxuICAgICAge1xuICAgICAgICBjYXJkaW5hbERpcmVjdGlvbkEgPSBJR2VvbWV0cnkuZ2V0Q2FyZGluYWxEaXJlY3Rpb24oc2xvcGVBLCBzbG9wZVByaW1lLCAyKTtcbiAgICAgICAgY2FyZGluYWxEaXJlY3Rpb25CID0gSUdlb21ldHJ5LmdldENhcmRpbmFsRGlyZWN0aW9uKHNsb3BlQiwgc2xvcGVQcmltZSwgNCk7XG4gICAgICB9XG4gICAgfVxuICAgIC8vY2FsY3VsYXRlIGNsaXBwaW5nIFBvaW50IGlmIGl0IGlzIG5vdCBmb3VuZCBiZWZvcmVcbiAgICBpZiAoIWNsaXBQb2ludEFGb3VuZClcbiAgICB7XG4gICAgICBzd2l0Y2ggKGNhcmRpbmFsRGlyZWN0aW9uQSlcbiAgICAgIHtcbiAgICAgICAgY2FzZSAxOlxuICAgICAgICAgIHRlbXBQb2ludEF5ID0gdG9wTGVmdEF5O1xuICAgICAgICAgIHRlbXBQb2ludEF4ID0gcDF4ICsgKC1oYWxmSGVpZ2h0QSkgLyBzbG9wZVByaW1lO1xuICAgICAgICAgIHJlc3VsdFswXSA9IHRlbXBQb2ludEF4O1xuICAgICAgICAgIHJlc3VsdFsxXSA9IHRlbXBQb2ludEF5O1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICBjYXNlIDI6XG4gICAgICAgICAgdGVtcFBvaW50QXggPSBib3R0b21SaWdodEF4O1xuICAgICAgICAgIHRlbXBQb2ludEF5ID0gcDF5ICsgaGFsZldpZHRoQSAqIHNsb3BlUHJpbWU7XG4gICAgICAgICAgcmVzdWx0WzBdID0gdGVtcFBvaW50QXg7XG4gICAgICAgICAgcmVzdWx0WzFdID0gdGVtcFBvaW50QXk7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIGNhc2UgMzpcbiAgICAgICAgICB0ZW1wUG9pbnRBeSA9IGJvdHRvbUxlZnRBeTtcbiAgICAgICAgICB0ZW1wUG9pbnRBeCA9IHAxeCArIGhhbGZIZWlnaHRBIC8gc2xvcGVQcmltZTtcbiAgICAgICAgICByZXN1bHRbMF0gPSB0ZW1wUG9pbnRBeDtcbiAgICAgICAgICByZXN1bHRbMV0gPSB0ZW1wUG9pbnRBeTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgY2FzZSA0OlxuICAgICAgICAgIHRlbXBQb2ludEF4ID0gYm90dG9tTGVmdEF4O1xuICAgICAgICAgIHRlbXBQb2ludEF5ID0gcDF5ICsgKC1oYWxmV2lkdGhBKSAqIHNsb3BlUHJpbWU7XG4gICAgICAgICAgcmVzdWx0WzBdID0gdGVtcFBvaW50QXg7XG4gICAgICAgICAgcmVzdWx0WzFdID0gdGVtcFBvaW50QXk7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICB9XG4gICAgfVxuICAgIGlmICghY2xpcFBvaW50QkZvdW5kKVxuICAgIHtcbiAgICAgIHN3aXRjaCAoY2FyZGluYWxEaXJlY3Rpb25CKVxuICAgICAge1xuICAgICAgICBjYXNlIDE6XG4gICAgICAgICAgdGVtcFBvaW50QnkgPSB0b3BMZWZ0Qnk7XG4gICAgICAgICAgdGVtcFBvaW50QnggPSBwMnggKyAoLWhhbGZIZWlnaHRCKSAvIHNsb3BlUHJpbWU7XG4gICAgICAgICAgcmVzdWx0WzJdID0gdGVtcFBvaW50Qng7XG4gICAgICAgICAgcmVzdWx0WzNdID0gdGVtcFBvaW50Qnk7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIGNhc2UgMjpcbiAgICAgICAgICB0ZW1wUG9pbnRCeCA9IGJvdHRvbVJpZ2h0Qng7XG4gICAgICAgICAgdGVtcFBvaW50QnkgPSBwMnkgKyBoYWxmV2lkdGhCICogc2xvcGVQcmltZTtcbiAgICAgICAgICByZXN1bHRbMl0gPSB0ZW1wUG9pbnRCeDtcbiAgICAgICAgICByZXN1bHRbM10gPSB0ZW1wUG9pbnRCeTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgY2FzZSAzOlxuICAgICAgICAgIHRlbXBQb2ludEJ5ID0gYm90dG9tTGVmdEJ5O1xuICAgICAgICAgIHRlbXBQb2ludEJ4ID0gcDJ4ICsgaGFsZkhlaWdodEIgLyBzbG9wZVByaW1lO1xuICAgICAgICAgIHJlc3VsdFsyXSA9IHRlbXBQb2ludEJ4O1xuICAgICAgICAgIHJlc3VsdFszXSA9IHRlbXBQb2ludEJ5O1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICBjYXNlIDQ6XG4gICAgICAgICAgdGVtcFBvaW50QnggPSBib3R0b21MZWZ0Qng7XG4gICAgICAgICAgdGVtcFBvaW50QnkgPSBwMnkgKyAoLWhhbGZXaWR0aEIpICogc2xvcGVQcmltZTtcbiAgICAgICAgICByZXN1bHRbMl0gPSB0ZW1wUG9pbnRCeDtcbiAgICAgICAgICByZXN1bHRbM10gPSB0ZW1wUG9pbnRCeTtcbiAgICAgICAgICBicmVhaztcbiAgICAgIH1cbiAgICB9XG4gIH1cbiAgcmV0dXJuIGZhbHNlO1xufVxuXG5JR2VvbWV0cnkuZ2V0Q2FyZGluYWxEaXJlY3Rpb24gPSBmdW5jdGlvbiAoc2xvcGUsIHNsb3BlUHJpbWUsIGxpbmUpXG57XG4gIGlmIChzbG9wZSA+IHNsb3BlUHJpbWUpXG4gIHtcbiAgICByZXR1cm4gbGluZTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICByZXR1cm4gMSArIGxpbmUgJSA0O1xuICB9XG59XG5cbklHZW9tZXRyeS5nZXRJbnRlcnNlY3Rpb24gPSBmdW5jdGlvbiAoczEsIHMyLCBmMSwgZjIpXG57XG4gIGlmIChmMiA9PSBudWxsKSB7XG4gICAgcmV0dXJuIElHZW9tZXRyeS5nZXRJbnRlcnNlY3Rpb24yKHMxLCBzMiwgZjEpO1xuICB9XG4gIHZhciB4MSA9IHMxLng7XG4gIHZhciB5MSA9IHMxLnk7XG4gIHZhciB4MiA9IHMyLng7XG4gIHZhciB5MiA9IHMyLnk7XG4gIHZhciB4MyA9IGYxLng7XG4gIHZhciB5MyA9IGYxLnk7XG4gIHZhciB4NCA9IGYyLng7XG4gIHZhciB5NCA9IGYyLnk7XG4gIHZhciB4LCB5OyAvLyBpbnRlcnNlY3Rpb24gcG9pbnRcbiAgdmFyIGExLCBhMiwgYjEsIGIyLCBjMSwgYzI7IC8vIGNvZWZmaWNpZW50cyBvZiBsaW5lIGVxbnMuXG4gIHZhciBkZW5vbTtcblxuICBhMSA9IHkyIC0geTE7XG4gIGIxID0geDEgLSB4MjtcbiAgYzEgPSB4MiAqIHkxIC0geDEgKiB5MjsgIC8vIHsgYTEqeCArIGIxKnkgKyBjMSA9IDAgaXMgbGluZSAxIH1cblxuICBhMiA9IHk0IC0geTM7XG4gIGIyID0geDMgLSB4NDtcbiAgYzIgPSB4NCAqIHkzIC0geDMgKiB5NDsgIC8vIHsgYTIqeCArIGIyKnkgKyBjMiA9IDAgaXMgbGluZSAyIH1cblxuICBkZW5vbSA9IGExICogYjIgLSBhMiAqIGIxO1xuXG4gIGlmIChkZW5vbSA9PSAwKVxuICB7XG4gICAgcmV0dXJuIG51bGw7XG4gIH1cblxuICB4ID0gKGIxICogYzIgLSBiMiAqIGMxKSAvIGRlbm9tO1xuICB5ID0gKGEyICogYzEgLSBhMSAqIGMyKSAvIGRlbm9tO1xuXG4gIHJldHVybiBuZXcgUG9pbnQoeCwgeSk7XG59XG5cbi8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG4vLyBTZWN0aW9uOiBDbGFzcyBDb25zdGFudHNcbi8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG4vKipcbiAqIFNvbWUgdXNlZnVsIHByZS1jYWxjdWxhdGVkIGNvbnN0YW50c1xuICovXG5JR2VvbWV0cnkuSEFMRl9QSSA9IDAuNSAqIE1hdGguUEk7XG5JR2VvbWV0cnkuT05FX0FORF9IQUxGX1BJID0gMS41ICogTWF0aC5QSTtcbklHZW9tZXRyeS5UV09fUEkgPSAyLjAgKiBNYXRoLlBJO1xuSUdlb21ldHJ5LlRIUkVFX1BJID0gMy4wICogTWF0aC5QSTtcblxubW9kdWxlLmV4cG9ydHMgPSBJR2VvbWV0cnk7XG4iLCJmdW5jdGlvbiBJTWF0aCgpIHtcbn1cblxuLyoqXG4gKiBUaGlzIG1ldGhvZCByZXR1cm5zIHRoZSBzaWduIG9mIHRoZSBpbnB1dCB2YWx1ZS5cbiAqL1xuSU1hdGguc2lnbiA9IGZ1bmN0aW9uICh2YWx1ZSkge1xuICBpZiAodmFsdWUgPiAwKVxuICB7XG4gICAgcmV0dXJuIDE7XG4gIH1cbiAgZWxzZSBpZiAodmFsdWUgPCAwKVxuICB7XG4gICAgcmV0dXJuIC0xO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIHJldHVybiAwO1xuICB9XG59XG5cbklNYXRoLmZsb29yID0gZnVuY3Rpb24gKHZhbHVlKSB7XG4gIHJldHVybiB2YWx1ZSA8IDAgPyBNYXRoLmNlaWwodmFsdWUpIDogTWF0aC5mbG9vcih2YWx1ZSk7XG59XG5cbklNYXRoLmNlaWwgPSBmdW5jdGlvbiAodmFsdWUpIHtcbiAgcmV0dXJuIHZhbHVlIDwgMCA/IE1hdGguZmxvb3IodmFsdWUpIDogTWF0aC5jZWlsKHZhbHVlKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBJTWF0aDtcbiIsImZ1bmN0aW9uIEludGVnZXIoKSB7XG59XG5cbkludGVnZXIuTUFYX1ZBTFVFID0gMjE0NzQ4MzY0NztcbkludGVnZXIuTUlOX1ZBTFVFID0gLTIxNDc0ODM2NDg7XG5cbm1vZHVsZS5leHBvcnRzID0gSW50ZWdlcjtcbiIsInZhciBMR3JhcGhPYmplY3QgPSByZXF1aXJlKCcuL0xHcmFwaE9iamVjdCcpO1xuXG5mdW5jdGlvbiBMRWRnZShzb3VyY2UsIHRhcmdldCwgdkVkZ2UpIHtcbiAgTEdyYXBoT2JqZWN0LmNhbGwodGhpcywgdkVkZ2UpO1xuXG4gIHRoaXMuaXNPdmVybGFwaW5nU291cmNlQW5kVGFyZ2V0ID0gZmFsc2U7XG4gIHRoaXMudkdyYXBoT2JqZWN0ID0gdkVkZ2U7XG4gIHRoaXMuYmVuZHBvaW50cyA9IFtdO1xuICB0aGlzLnNvdXJjZSA9IHNvdXJjZTtcbiAgdGhpcy50YXJnZXQgPSB0YXJnZXQ7XG59XG5cbkxFZGdlLnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoTEdyYXBoT2JqZWN0LnByb3RvdHlwZSk7XG5cbmZvciAodmFyIHByb3AgaW4gTEdyYXBoT2JqZWN0KSB7XG4gIExFZGdlW3Byb3BdID0gTEdyYXBoT2JqZWN0W3Byb3BdO1xufVxuXG5MRWRnZS5wcm90b3R5cGUuZ2V0U291cmNlID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuc291cmNlO1xufTtcblxuTEVkZ2UucHJvdG90eXBlLmdldFRhcmdldCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnRhcmdldDtcbn07XG5cbkxFZGdlLnByb3RvdHlwZS5pc0ludGVyR3JhcGggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5pc0ludGVyR3JhcGg7XG59O1xuXG5MRWRnZS5wcm90b3R5cGUuZ2V0TGVuZ3RoID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMubGVuZ3RoO1xufTtcblxuTEVkZ2UucHJvdG90eXBlLmlzT3ZlcmxhcGluZ1NvdXJjZUFuZFRhcmdldCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmlzT3ZlcmxhcGluZ1NvdXJjZUFuZFRhcmdldDtcbn07XG5cbkxFZGdlLnByb3RvdHlwZS5nZXRCZW5kcG9pbnRzID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuYmVuZHBvaW50cztcbn07XG5cbkxFZGdlLnByb3RvdHlwZS5nZXRMY2EgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sY2E7XG59O1xuXG5MRWRnZS5wcm90b3R5cGUuZ2V0U291cmNlSW5MY2EgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5zb3VyY2VJbkxjYTtcbn07XG5cbkxFZGdlLnByb3RvdHlwZS5nZXRUYXJnZXRJbkxjYSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnRhcmdldEluTGNhO1xufTtcblxuTEVkZ2UucHJvdG90eXBlLmdldE90aGVyRW5kID0gZnVuY3Rpb24gKG5vZGUpXG57XG4gIGlmICh0aGlzLnNvdXJjZSA9PT0gbm9kZSlcbiAge1xuICAgIHJldHVybiB0aGlzLnRhcmdldDtcbiAgfVxuICBlbHNlIGlmICh0aGlzLnRhcmdldCA9PT0gbm9kZSlcbiAge1xuICAgIHJldHVybiB0aGlzLnNvdXJjZTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICB0aHJvdyBcIk5vZGUgaXMgbm90IGluY2lkZW50IHdpdGggdGhpcyBlZGdlXCI7XG4gIH1cbn1cblxuTEVkZ2UucHJvdG90eXBlLmdldE90aGVyRW5kSW5HcmFwaCA9IGZ1bmN0aW9uIChub2RlLCBncmFwaClcbntcbiAgdmFyIG90aGVyRW5kID0gdGhpcy5nZXRPdGhlckVuZChub2RlKTtcbiAgdmFyIHJvb3QgPSBncmFwaC5nZXRHcmFwaE1hbmFnZXIoKS5nZXRSb290KCk7XG5cbiAgd2hpbGUgKHRydWUpXG4gIHtcbiAgICBpZiAob3RoZXJFbmQuZ2V0T3duZXIoKSA9PSBncmFwaClcbiAgICB7XG4gICAgICByZXR1cm4gb3RoZXJFbmQ7XG4gICAgfVxuXG4gICAgaWYgKG90aGVyRW5kLmdldE93bmVyKCkgPT0gcm9vdClcbiAgICB7XG4gICAgICBicmVhaztcbiAgICB9XG5cbiAgICBvdGhlckVuZCA9IG90aGVyRW5kLmdldE93bmVyKCkuZ2V0UGFyZW50KCk7XG4gIH1cblxuICByZXR1cm4gbnVsbDtcbn07XG5cbkxFZGdlLnByb3RvdHlwZS51cGRhdGVMZW5ndGggPSBmdW5jdGlvbiAoKVxue1xuICB2YXIgY2xpcFBvaW50Q29vcmRpbmF0ZXMgPSBuZXcgQXJyYXkoNCk7XG5cbiAgdGhpcy5pc092ZXJsYXBpbmdTb3VyY2VBbmRUYXJnZXQgPVxuICAgICAgICAgIElHZW9tZXRyeS5nZXRJbnRlcnNlY3Rpb24odGhpcy50YXJnZXQuZ2V0UmVjdCgpLFxuICAgICAgICAgICAgICAgICAgdGhpcy5zb3VyY2UuZ2V0UmVjdCgpLFxuICAgICAgICAgICAgICAgICAgY2xpcFBvaW50Q29vcmRpbmF0ZXMpO1xuXG4gIGlmICghdGhpcy5pc092ZXJsYXBpbmdTb3VyY2VBbmRUYXJnZXQpXG4gIHtcbiAgICB0aGlzLmxlbmd0aFggPSBjbGlwUG9pbnRDb29yZGluYXRlc1swXSAtIGNsaXBQb2ludENvb3JkaW5hdGVzWzJdO1xuICAgIHRoaXMubGVuZ3RoWSA9IGNsaXBQb2ludENvb3JkaW5hdGVzWzFdIC0gY2xpcFBvaW50Q29vcmRpbmF0ZXNbM107XG5cbiAgICBpZiAoTWF0aC5hYnModGhpcy5sZW5ndGhYKSA8IDEuMClcbiAgICB7XG4gICAgICB0aGlzLmxlbmd0aFggPSBJTWF0aC5zaWduKHRoaXMubGVuZ3RoWCk7XG4gICAgfVxuXG4gICAgaWYgKE1hdGguYWJzKHRoaXMubGVuZ3RoWSkgPCAxLjApXG4gICAge1xuICAgICAgdGhpcy5sZW5ndGhZID0gSU1hdGguc2lnbih0aGlzLmxlbmd0aFkpO1xuICAgIH1cblxuICAgIHRoaXMubGVuZ3RoID0gTWF0aC5zcXJ0KFxuICAgICAgICAgICAgdGhpcy5sZW5ndGhYICogdGhpcy5sZW5ndGhYICsgdGhpcy5sZW5ndGhZICogdGhpcy5sZW5ndGhZKTtcbiAgfVxufTtcblxuTEVkZ2UucHJvdG90eXBlLnVwZGF0ZUxlbmd0aFNpbXBsZSA9IGZ1bmN0aW9uICgpXG57XG4gIHRoaXMubGVuZ3RoWCA9IHRoaXMudGFyZ2V0LmdldENlbnRlclgoKSAtIHRoaXMuc291cmNlLmdldENlbnRlclgoKTtcbiAgdGhpcy5sZW5ndGhZID0gdGhpcy50YXJnZXQuZ2V0Q2VudGVyWSgpIC0gdGhpcy5zb3VyY2UuZ2V0Q2VudGVyWSgpO1xuXG4gIGlmIChNYXRoLmFicyh0aGlzLmxlbmd0aFgpIDwgMS4wKVxuICB7XG4gICAgdGhpcy5sZW5ndGhYID0gSU1hdGguc2lnbih0aGlzLmxlbmd0aFgpO1xuICB9XG5cbiAgaWYgKE1hdGguYWJzKHRoaXMubGVuZ3RoWSkgPCAxLjApXG4gIHtcbiAgICB0aGlzLmxlbmd0aFkgPSBJTWF0aC5zaWduKHRoaXMubGVuZ3RoWSk7XG4gIH1cblxuICB0aGlzLmxlbmd0aCA9IE1hdGguc3FydChcbiAgICAgICAgICB0aGlzLmxlbmd0aFggKiB0aGlzLmxlbmd0aFggKyB0aGlzLmxlbmd0aFkgKiB0aGlzLmxlbmd0aFkpO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IExFZGdlO1xuIiwidmFyIExHcmFwaE9iamVjdCA9IHJlcXVpcmUoJy4vTEdyYXBoT2JqZWN0Jyk7XG52YXIgSW50ZWdlciA9IHJlcXVpcmUoJy4vSW50ZWdlcicpO1xudmFyIExheW91dENvbnN0YW50cyA9IHJlcXVpcmUoJy4vTGF5b3V0Q29uc3RhbnRzJyk7XG52YXIgTEdyYXBoTWFuYWdlciA9IHJlcXVpcmUoJy4vTEdyYXBoTWFuYWdlcicpO1xudmFyIExOb2RlID0gcmVxdWlyZSgnLi9MTm9kZScpO1xuXG5mdW5jdGlvbiBMR3JhcGgocGFyZW50LCBvYmoyLCB2R3JhcGgpIHtcbiAgTEdyYXBoT2JqZWN0LmNhbGwodGhpcywgdkdyYXBoKTtcbiAgdGhpcy5lc3RpbWF0ZWRTaXplID0gSW50ZWdlci5NSU5fVkFMVUU7XG4gIHRoaXMubWFyZ2luID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBUEhfTUFSR0lOO1xuICB0aGlzLmVkZ2VzID0gW107XG4gIHRoaXMubm9kZXMgPSBbXTtcbiAgdGhpcy5pc0Nvbm5lY3RlZCA9IGZhbHNlO1xuICB0aGlzLnBhcmVudCA9IHBhcmVudDtcblxuICBpZiAob2JqMiAhPSBudWxsICYmIG9iajIgaW5zdGFuY2VvZiBMR3JhcGhNYW5hZ2VyKSB7XG4gICAgdGhpcy5ncmFwaE1hbmFnZXIgPSBvYmoyO1xuICB9XG4gIGVsc2UgaWYgKG9iajIgIT0gbnVsbCAmJiBvYmoyIGluc3RhbmNlb2YgTGF5b3V0KSB7XG4gICAgdGhpcy5ncmFwaE1hbmFnZXIgPSBvYmoyLmdyYXBoTWFuYWdlcjtcbiAgfVxufVxuXG5MR3JhcGgucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMR3JhcGhPYmplY3QucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gTEdyYXBoT2JqZWN0KSB7XG4gIExHcmFwaFtwcm9wXSA9IExHcmFwaE9iamVjdFtwcm9wXTtcbn1cblxuTEdyYXBoLnByb3RvdHlwZS5nZXROb2RlcyA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIHRoaXMubm9kZXM7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmdldEVkZ2VzID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gdGhpcy5lZGdlcztcbn07XG5cbkxHcmFwaC5wcm90b3R5cGUuZ2V0R3JhcGhNYW5hZ2VyID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuZ3JhcGhNYW5hZ2VyO1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRQYXJlbnQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5wYXJlbnQ7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmdldExlZnQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sZWZ0O1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRSaWdodCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnJpZ2h0O1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRUb3AgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy50b3A7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmdldEJvdHRvbSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmJvdHRvbTtcbn07XG5cbkxHcmFwaC5wcm90b3R5cGUuaXNDb25uZWN0ZWQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5pc0Nvbm5lY3RlZDtcbn07XG5cbkxHcmFwaC5wcm90b3R5cGUuYWRkID0gZnVuY3Rpb24gKG9iajEsIHNvdXJjZU5vZGUsIHRhcmdldE5vZGUpIHtcbiAgaWYgKHNvdXJjZU5vZGUgPT0gbnVsbCAmJiB0YXJnZXROb2RlID09IG51bGwpIHtcbiAgICB2YXIgbmV3Tm9kZSA9IG9iajE7XG4gICAgaWYgKHRoaXMuZ3JhcGhNYW5hZ2VyID09IG51bGwpIHtcbiAgICAgIHRocm93IFwiR3JhcGggaGFzIG5vIGdyYXBoIG1nciFcIjtcbiAgICB9XG4gICAgaWYgKHRoaXMuZ2V0Tm9kZXMoKS5pbmRleE9mKG5ld05vZGUpID4gLTEpIHtcbiAgICAgIHRocm93IFwiTm9kZSBhbHJlYWR5IGluIGdyYXBoIVwiO1xuICAgIH1cbiAgICBuZXdOb2RlLm93bmVyID0gdGhpcztcbiAgICB0aGlzLmdldE5vZGVzKCkucHVzaChuZXdOb2RlKTtcblxuICAgIHJldHVybiBuZXdOb2RlO1xuICB9XG4gIGVsc2Uge1xuICAgIHZhciBuZXdFZGdlID0gb2JqMTtcbiAgICBpZiAoISh0aGlzLmdldE5vZGVzKCkuaW5kZXhPZihzb3VyY2VOb2RlKSA+IC0xICYmICh0aGlzLmdldE5vZGVzKCkuaW5kZXhPZih0YXJnZXROb2RlKSkgPiAtMSkpIHtcbiAgICAgIHRocm93IFwiU291cmNlIG9yIHRhcmdldCBub3QgaW4gZ3JhcGghXCI7XG4gICAgfVxuXG4gICAgaWYgKCEoc291cmNlTm9kZS5vd25lciA9PSB0YXJnZXROb2RlLm93bmVyICYmIHNvdXJjZU5vZGUub3duZXIgPT0gdGhpcykpIHtcbiAgICAgIHRocm93IFwiQm90aCBvd25lcnMgbXVzdCBiZSB0aGlzIGdyYXBoIVwiO1xuICAgIH1cblxuICAgIGlmIChzb3VyY2VOb2RlLm93bmVyICE9IHRhcmdldE5vZGUub3duZXIpXG4gICAge1xuICAgICAgcmV0dXJuIG51bGw7XG4gICAgfVxuXG4gICAgLy8gc2V0IHNvdXJjZSBhbmQgdGFyZ2V0XG4gICAgbmV3RWRnZS5zb3VyY2UgPSBzb3VyY2VOb2RlO1xuICAgIG5ld0VkZ2UudGFyZ2V0ID0gdGFyZ2V0Tm9kZTtcblxuICAgIC8vIHNldCBhcyBpbnRyYS1ncmFwaCBlZGdlXG4gICAgbmV3RWRnZS5pc0ludGVyR3JhcGggPSBmYWxzZTtcblxuICAgIC8vIGFkZCB0byBncmFwaCBlZGdlIGxpc3RcbiAgICB0aGlzLmdldEVkZ2VzKCkucHVzaChuZXdFZGdlKTtcblxuICAgIC8vIGFkZCB0byBpbmNpZGVuY3kgbGlzdHNcbiAgICBzb3VyY2VOb2RlLmVkZ2VzLnB1c2gobmV3RWRnZSk7XG5cbiAgICBpZiAodGFyZ2V0Tm9kZSAhPSBzb3VyY2VOb2RlKVxuICAgIHtcbiAgICAgIHRhcmdldE5vZGUuZWRnZXMucHVzaChuZXdFZGdlKTtcbiAgICB9XG5cbiAgICByZXR1cm4gbmV3RWRnZTtcbiAgfVxufTtcblxuTEdyYXBoLnByb3RvdHlwZS5yZW1vdmUgPSBmdW5jdGlvbiAob2JqKSB7XG4gIHZhciBub2RlID0gb2JqO1xuICBpZiAob2JqIGluc3RhbmNlb2YgTE5vZGUpIHtcbiAgICBpZiAobm9kZSA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcIk5vZGUgaXMgbnVsbCFcIjtcbiAgICB9XG4gICAgaWYgKCEobm9kZS5vd25lciAhPSBudWxsICYmIG5vZGUub3duZXIgPT0gdGhpcykpIHtcbiAgICAgIHRocm93IFwiT3duZXIgZ3JhcGggaXMgaW52YWxpZCFcIjtcbiAgICB9XG4gICAgaWYgKHRoaXMuZ3JhcGhNYW5hZ2VyID09IG51bGwpIHtcbiAgICAgIHRocm93IFwiT3duZXIgZ3JhcGggbWFuYWdlciBpcyBpbnZhbGlkIVwiO1xuICAgIH1cbiAgICAvLyByZW1vdmUgaW5jaWRlbnQgZWRnZXMgZmlyc3QgKG1ha2UgYSBjb3B5IHRvIGRvIGl0IHNhZmVseSlcbiAgICB2YXIgZWRnZXNUb0JlUmVtb3ZlZCA9IG5vZGUuZWRnZXMuc2xpY2UoKTtcbiAgICB2YXIgZWRnZTtcbiAgICB2YXIgcyA9IGVkZ2VzVG9CZVJlbW92ZWQubGVuZ3RoO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICAgIHtcbiAgICAgIGVkZ2UgPSBlZGdlc1RvQmVSZW1vdmVkW2ldO1xuXG4gICAgICBpZiAoZWRnZS5pc0ludGVyR3JhcGgpXG4gICAgICB7XG4gICAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlbW92ZShlZGdlKTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgZWRnZS5zb3VyY2Uub3duZXIucmVtb3ZlKGVkZ2UpO1xuICAgICAgfVxuICAgIH1cblxuICAgIC8vIG5vdyB0aGUgbm9kZSBpdHNlbGZcbiAgICB2YXIgaW5kZXggPSB0aGlzLm5vZGVzLmluZGV4T2Yobm9kZSk7XG4gICAgaWYgKGluZGV4ID09IC0xKSB7XG4gICAgICB0aHJvdyBcIk5vZGUgbm90IGluIG93bmVyIG5vZGUgbGlzdCFcIjtcbiAgICB9XG5cbiAgICB0aGlzLm5vZGVzLnNwbGljZShpbmRleCwgMSk7XG4gIH1cbiAgZWxzZSBpZiAob2JqIGluc3RhbmNlb2YgTEVkZ2UpIHtcbiAgICB2YXIgZWRnZSA9IG9iajtcbiAgICBpZiAoZWRnZSA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcIkVkZ2UgaXMgbnVsbCFcIjtcbiAgICB9XG4gICAgaWYgKCEoZWRnZS5zb3VyY2UgIT0gbnVsbCAmJiBlZGdlLnRhcmdldCAhPSBudWxsKSkge1xuICAgICAgdGhyb3cgXCJTb3VyY2UgYW5kL29yIHRhcmdldCBpcyBudWxsIVwiO1xuICAgIH1cbiAgICBpZiAoIShlZGdlLnNvdXJjZS5vd25lciAhPSBudWxsICYmIGVkZ2UudGFyZ2V0Lm93bmVyICE9IG51bGwgJiZcbiAgICAgICAgICAgIGVkZ2Uuc291cmNlLm93bmVyID09IHRoaXMgJiYgZWRnZS50YXJnZXQub3duZXIgPT0gdGhpcykpIHtcbiAgICAgIHRocm93IFwiU291cmNlIGFuZC9vciB0YXJnZXQgb3duZXIgaXMgaW52YWxpZCFcIjtcbiAgICB9XG5cbiAgICB2YXIgc291cmNlSW5kZXggPSBlZGdlLnNvdXJjZS5lZGdlcy5pbmRleE9mKGVkZ2UpO1xuICAgIHZhciB0YXJnZXRJbmRleCA9IGVkZ2UudGFyZ2V0LmVkZ2VzLmluZGV4T2YoZWRnZSk7XG4gICAgaWYgKCEoc291cmNlSW5kZXggPiAtMSAmJiB0YXJnZXRJbmRleCA+IC0xKSkge1xuICAgICAgdGhyb3cgXCJTb3VyY2UgYW5kL29yIHRhcmdldCBkb2Vzbid0IGtub3cgdGhpcyBlZGdlIVwiO1xuICAgIH1cblxuICAgIGVkZ2Uuc291cmNlLmVkZ2VzLnNwbGljZShzb3VyY2VJbmRleCwgMSk7XG5cbiAgICBpZiAoZWRnZS50YXJnZXQgIT0gZWRnZS5zb3VyY2UpXG4gICAge1xuICAgICAgZWRnZS50YXJnZXQuZWRnZXMuc3BsaWNlKHRhcmdldEluZGV4LCAxKTtcbiAgICB9XG5cbiAgICB2YXIgaW5kZXggPSBlZGdlLnNvdXJjZS5vd25lci5nZXRFZGdlcygpLmluZGV4T2YoZWRnZSk7XG4gICAgaWYgKGluZGV4ID09IC0xKSB7XG4gICAgICB0aHJvdyBcIk5vdCBpbiBvd25lcidzIGVkZ2UgbGlzdCFcIjtcbiAgICB9XG5cbiAgICBlZGdlLnNvdXJjZS5vd25lci5nZXRFZGdlcygpLnNwbGljZShpbmRleCwgMSk7XG4gIH1cbn07XG5cbkxHcmFwaC5wcm90b3R5cGUudXBkYXRlTGVmdFRvcCA9IGZ1bmN0aW9uICgpXG57XG4gIHZhciB0b3AgPSBJbnRlZ2VyLk1BWF9WQUxVRTtcbiAgdmFyIGxlZnQgPSBJbnRlZ2VyLk1BWF9WQUxVRTtcbiAgdmFyIG5vZGVUb3A7XG4gIHZhciBub2RlTGVmdDtcblxuICB2YXIgbm9kZXMgPSB0aGlzLmdldE5vZGVzKCk7XG4gIHZhciBzID0gbm9kZXMubGVuZ3RoO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICB7XG4gICAgdmFyIGxOb2RlID0gbm9kZXNbaV07XG4gICAgbm9kZVRvcCA9IE1hdGguZmxvb3IobE5vZGUuZ2V0VG9wKCkpO1xuICAgIG5vZGVMZWZ0ID0gTWF0aC5mbG9vcihsTm9kZS5nZXRMZWZ0KCkpO1xuXG4gICAgaWYgKHRvcCA+IG5vZGVUb3ApXG4gICAge1xuICAgICAgdG9wID0gbm9kZVRvcDtcbiAgICB9XG5cbiAgICBpZiAobGVmdCA+IG5vZGVMZWZ0KVxuICAgIHtcbiAgICAgIGxlZnQgPSBub2RlTGVmdDtcbiAgICB9XG4gIH1cblxuICAvLyBEbyB3ZSBoYXZlIGFueSBub2RlcyBpbiB0aGlzIGdyYXBoP1xuICBpZiAodG9wID09IEludGVnZXIuTUFYX1ZBTFVFKVxuICB7XG4gICAgcmV0dXJuIG51bGw7XG4gIH1cblxuICB0aGlzLmxlZnQgPSBsZWZ0IC0gdGhpcy5tYXJnaW47XG4gIHRoaXMudG9wID0gdG9wIC0gdGhpcy5tYXJnaW47XG5cbiAgLy8gQXBwbHkgdGhlIG1hcmdpbnMgYW5kIHJldHVybiB0aGUgcmVzdWx0XG4gIHJldHVybiBuZXcgUG9pbnQodGhpcy5sZWZ0LCB0aGlzLnRvcCk7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLnVwZGF0ZUJvdW5kcyA9IGZ1bmN0aW9uIChyZWN1cnNpdmUpXG57XG4gIC8vIGNhbGN1bGF0ZSBib3VuZHNcbiAgdmFyIGxlZnQgPSBJbnRlZ2VyLk1BWF9WQUxVRTtcbiAgdmFyIHJpZ2h0ID0gLUludGVnZXIuTUFYX1ZBTFVFO1xuICB2YXIgdG9wID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gIHZhciBib3R0b20gPSAtSW50ZWdlci5NQVhfVkFMVUU7XG4gIHZhciBub2RlTGVmdDtcbiAgdmFyIG5vZGVSaWdodDtcbiAgdmFyIG5vZGVUb3A7XG4gIHZhciBub2RlQm90dG9tO1xuXG4gIHZhciBub2RlcyA9IHRoaXMubm9kZXM7XG4gIHZhciBzID0gbm9kZXMubGVuZ3RoO1xuICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAge1xuICAgIHZhciBsTm9kZSA9IG5vZGVzW2ldO1xuXG4gICAgaWYgKHJlY3Vyc2l2ZSAmJiBsTm9kZS5jaGlsZCAhPSBudWxsKVxuICAgIHtcbiAgICAgIGxOb2RlLnVwZGF0ZUJvdW5kcygpO1xuICAgIH1cbiAgICBub2RlTGVmdCA9IE1hdGguZmxvb3IobE5vZGUuZ2V0TGVmdCgpKTtcbiAgICBub2RlUmlnaHQgPSBNYXRoLmZsb29yKGxOb2RlLmdldFJpZ2h0KCkpO1xuICAgIG5vZGVUb3AgPSBNYXRoLmZsb29yKGxOb2RlLmdldFRvcCgpKTtcbiAgICBub2RlQm90dG9tID0gTWF0aC5mbG9vcihsTm9kZS5nZXRCb3R0b20oKSk7XG5cbiAgICBpZiAobGVmdCA+IG5vZGVMZWZ0KVxuICAgIHtcbiAgICAgIGxlZnQgPSBub2RlTGVmdDtcbiAgICB9XG5cbiAgICBpZiAocmlnaHQgPCBub2RlUmlnaHQpXG4gICAge1xuICAgICAgcmlnaHQgPSBub2RlUmlnaHQ7XG4gICAgfVxuXG4gICAgaWYgKHRvcCA+IG5vZGVUb3ApXG4gICAge1xuICAgICAgdG9wID0gbm9kZVRvcDtcbiAgICB9XG5cbiAgICBpZiAoYm90dG9tIDwgbm9kZUJvdHRvbSlcbiAgICB7XG4gICAgICBib3R0b20gPSBub2RlQm90dG9tO1xuICAgIH1cbiAgfVxuXG4gIHZhciBib3VuZGluZ1JlY3QgPSBuZXcgUmVjdGFuZ2xlRChsZWZ0LCB0b3AsIHJpZ2h0IC0gbGVmdCwgYm90dG9tIC0gdG9wKTtcbiAgaWYgKGxlZnQgPT0gSW50ZWdlci5NQVhfVkFMVUUpXG4gIHtcbiAgICB0aGlzLmxlZnQgPSBNYXRoLmZsb29yKHRoaXMucGFyZW50LmdldExlZnQoKSk7XG4gICAgdGhpcy5yaWdodCA9IE1hdGguZmxvb3IodGhpcy5wYXJlbnQuZ2V0UmlnaHQoKSk7XG4gICAgdGhpcy50b3AgPSBNYXRoLmZsb29yKHRoaXMucGFyZW50LmdldFRvcCgpKTtcbiAgICB0aGlzLmJvdHRvbSA9IE1hdGguZmxvb3IodGhpcy5wYXJlbnQuZ2V0Qm90dG9tKCkpO1xuICB9XG5cbiAgdGhpcy5sZWZ0ID0gYm91bmRpbmdSZWN0LnggLSB0aGlzLm1hcmdpbjtcbiAgdGhpcy5yaWdodCA9IGJvdW5kaW5nUmVjdC54ICsgYm91bmRpbmdSZWN0LndpZHRoICsgdGhpcy5tYXJnaW47XG4gIHRoaXMudG9wID0gYm91bmRpbmdSZWN0LnkgLSB0aGlzLm1hcmdpbjtcbiAgdGhpcy5ib3R0b20gPSBib3VuZGluZ1JlY3QueSArIGJvdW5kaW5nUmVjdC5oZWlnaHQgKyB0aGlzLm1hcmdpbjtcbn07XG5cbkxHcmFwaC5jYWxjdWxhdGVCb3VuZHMgPSBmdW5jdGlvbiAobm9kZXMpXG57XG4gIHZhciBsZWZ0ID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gIHZhciByaWdodCA9IC1JbnRlZ2VyLk1BWF9WQUxVRTtcbiAgdmFyIHRvcCA9IEludGVnZXIuTUFYX1ZBTFVFO1xuICB2YXIgYm90dG9tID0gLUludGVnZXIuTUFYX1ZBTFVFO1xuICB2YXIgbm9kZUxlZnQ7XG4gIHZhciBub2RlUmlnaHQ7XG4gIHZhciBub2RlVG9wO1xuICB2YXIgbm9kZUJvdHRvbTtcblxuICB2YXIgcyA9IG5vZGVzLmxlbmd0aDtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAge1xuICAgIHZhciBsTm9kZSA9IG5vZGVzW2ldO1xuICAgIG5vZGVMZWZ0ID0gTWF0aC5mbG9vcihsTm9kZS5nZXRMZWZ0KCkpO1xuICAgIG5vZGVSaWdodCA9IE1hdGguZmxvb3IobE5vZGUuZ2V0UmlnaHQoKSk7XG4gICAgbm9kZVRvcCA9IE1hdGguZmxvb3IobE5vZGUuZ2V0VG9wKCkpO1xuICAgIG5vZGVCb3R0b20gPSBNYXRoLmZsb29yKGxOb2RlLmdldEJvdHRvbSgpKTtcblxuICAgIGlmIChsZWZ0ID4gbm9kZUxlZnQpXG4gICAge1xuICAgICAgbGVmdCA9IG5vZGVMZWZ0O1xuICAgIH1cblxuICAgIGlmIChyaWdodCA8IG5vZGVSaWdodClcbiAgICB7XG4gICAgICByaWdodCA9IG5vZGVSaWdodDtcbiAgICB9XG5cbiAgICBpZiAodG9wID4gbm9kZVRvcClcbiAgICB7XG4gICAgICB0b3AgPSBub2RlVG9wO1xuICAgIH1cblxuICAgIGlmIChib3R0b20gPCBub2RlQm90dG9tKVxuICAgIHtcbiAgICAgIGJvdHRvbSA9IG5vZGVCb3R0b207XG4gICAgfVxuICB9XG5cbiAgdmFyIGJvdW5kaW5nUmVjdCA9IG5ldyBSZWN0YW5nbGVEKGxlZnQsIHRvcCwgcmlnaHQgLSBsZWZ0LCBib3R0b20gLSB0b3ApO1xuXG4gIHJldHVybiBib3VuZGluZ1JlY3Q7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmdldEluY2x1c2lvblRyZWVEZXB0aCA9IGZ1bmN0aW9uICgpXG57XG4gIGlmICh0aGlzID09IHRoaXMuZ3JhcGhNYW5hZ2VyLmdldFJvb3QoKSlcbiAge1xuICAgIHJldHVybiAxO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIHJldHVybiB0aGlzLnBhcmVudC5nZXRJbmNsdXNpb25UcmVlRGVwdGgoKTtcbiAgfVxufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRFc3RpbWF0ZWRTaXplID0gZnVuY3Rpb24gKClcbntcbiAgaWYgKHRoaXMuZXN0aW1hdGVkU2l6ZSA9PSBJbnRlZ2VyLk1JTl9WQUxVRSkge1xuICAgIHRocm93IFwiYXNzZXJ0IGZhaWxlZFwiO1xuICB9XG4gIHJldHVybiB0aGlzLmVzdGltYXRlZFNpemU7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmNhbGNFc3RpbWF0ZWRTaXplID0gZnVuY3Rpb24gKClcbntcbiAgdmFyIHNpemUgPSAwO1xuICB2YXIgbm9kZXMgPSB0aGlzLm5vZGVzO1xuICB2YXIgcyA9IG5vZGVzLmxlbmd0aDtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAge1xuICAgIHZhciBsTm9kZSA9IG5vZGVzW2ldO1xuICAgIHNpemUgKz0gbE5vZGUuY2FsY0VzdGltYXRlZFNpemUoKTtcbiAgfVxuXG4gIGlmIChzaXplID09IDApXG4gIHtcbiAgICB0aGlzLmVzdGltYXRlZFNpemUgPSBMYXlvdXRDb25zdGFudHMuRU1QVFlfQ09NUE9VTkRfTk9ERV9TSVpFO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIHRoaXMuZXN0aW1hdGVkU2l6ZSA9IE1hdGguZmxvb3Ioc2l6ZSAvIE1hdGguc3FydCh0aGlzLm5vZGVzLmxlbmd0aCkpO1xuICB9XG5cbiAgcmV0dXJuIE1hdGguZmxvb3IodGhpcy5lc3RpbWF0ZWRTaXplKTtcbn07XG5cbkxHcmFwaC5wcm90b3R5cGUudXBkYXRlQ29ubmVjdGVkID0gZnVuY3Rpb24gKClcbntcbiAgaWYgKHRoaXMubm9kZXMubGVuZ3RoID09IDApXG4gIHtcbiAgICB0aGlzLmlzQ29ubmVjdGVkID0gdHJ1ZTtcbiAgICByZXR1cm47XG4gIH1cblxuICB2YXIgdG9CZVZpc2l0ZWQgPSBbXTtcbiAgdmFyIHZpc2l0ZWQgPSBuZXcgSGFzaFNldCgpO1xuICB2YXIgY3VycmVudE5vZGUgPSB0aGlzLm5vZGVzWzBdO1xuICB2YXIgbmVpZ2hib3JFZGdlcztcbiAgdmFyIGN1cnJlbnROZWlnaGJvcjtcbiAgdG9CZVZpc2l0ZWQgPSB0b0JlVmlzaXRlZC5jb25jYXQoY3VycmVudE5vZGUud2l0aENoaWxkcmVuKCkpO1xuXG4gIHdoaWxlICh0b0JlVmlzaXRlZC5sZW5ndGggPiAwKVxuICB7XG4gICAgY3VycmVudE5vZGUgPSB0b0JlVmlzaXRlZC5zaGlmdCgpO1xuICAgIHZpc2l0ZWQuYWRkKGN1cnJlbnROb2RlKTtcblxuICAgIC8vIFRyYXZlcnNlIGFsbCBuZWlnaGJvcnMgb2YgdGhpcyBub2RlXG4gICAgbmVpZ2hib3JFZGdlcyA9IGN1cnJlbnROb2RlLmdldEVkZ2VzKCk7XG4gICAgdmFyIHMgPSBuZWlnaGJvckVkZ2VzLmxlbmd0aDtcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAgICB7XG4gICAgICB2YXIgbmVpZ2hib3JFZGdlID0gbmVpZ2hib3JFZGdlc1tpXTtcbiAgICAgIGN1cnJlbnROZWlnaGJvciA9XG4gICAgICAgICAgICAgIG5laWdoYm9yRWRnZS5nZXRPdGhlckVuZEluR3JhcGgoY3VycmVudE5vZGUsIHRoaXMpO1xuXG4gICAgICAvLyBBZGQgdW52aXNpdGVkIG5laWdoYm9ycyB0byB0aGUgbGlzdCB0byB2aXNpdFxuICAgICAgaWYgKGN1cnJlbnROZWlnaGJvciAhPSBudWxsICYmXG4gICAgICAgICAgICAgICF2aXNpdGVkLmNvbnRhaW5zKGN1cnJlbnROZWlnaGJvcikpXG4gICAgICB7XG4gICAgICAgIHRvQmVWaXNpdGVkID0gdG9CZVZpc2l0ZWQuY29uY2F0KGN1cnJlbnROZWlnaGJvci53aXRoQ2hpbGRyZW4oKSk7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgdGhpcy5pc0Nvbm5lY3RlZCA9IGZhbHNlO1xuXG4gIGlmICh2aXNpdGVkLnNpemUoKSA+PSB0aGlzLm5vZGVzLmxlbmd0aClcbiAge1xuICAgIHZhciBub09mVmlzaXRlZEluVGhpc0dyYXBoID0gMDtcblxuICAgIHZhciBzID0gdmlzaXRlZC5zaXplKCk7XG4gICAgZm9yICh2YXIgdmlzaXRlZElkIGluIHZpc2l0ZWQuc2V0KVxuICAgIHtcbiAgICAgIHZhciB2aXNpdGVkTm9kZSA9IHZpc2l0ZWQuc2V0W3Zpc2l0ZWRJZF07XG4gICAgICBpZiAodmlzaXRlZE5vZGUub3duZXIgPT0gdGhpcylcbiAgICAgIHtcbiAgICAgICAgbm9PZlZpc2l0ZWRJblRoaXNHcmFwaCsrO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChub09mVmlzaXRlZEluVGhpc0dyYXBoID09IHRoaXMubm9kZXMubGVuZ3RoKVxuICAgIHtcbiAgICAgIHRoaXMuaXNDb25uZWN0ZWQgPSB0cnVlO1xuICAgIH1cbiAgfVxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBMR3JhcGg7XG4iLCJmdW5jdGlvbiBMR3JhcGhNYW5hZ2VyKGxheW91dCkge1xuICB0aGlzLmxheW91dCA9IGxheW91dDtcblxuICB0aGlzLmdyYXBocyA9IFtdO1xuICB0aGlzLmVkZ2VzID0gW107XG59XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmFkZFJvb3QgPSBmdW5jdGlvbiAoKVxue1xuICB2YXIgbmdyYXBoID0gdGhpcy5sYXlvdXQubmV3R3JhcGgoKTtcbiAgdmFyIG5ub2RlID0gdGhpcy5sYXlvdXQubmV3Tm9kZShudWxsKTtcbiAgdmFyIHJvb3QgPSB0aGlzLmFkZChuZ3JhcGgsIG5ub2RlKTtcbiAgdGhpcy5zZXRSb290R3JhcGgocm9vdCk7XG4gIHJldHVybiB0aGlzLnJvb3RHcmFwaDtcbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmFkZCA9IGZ1bmN0aW9uIChuZXdHcmFwaCwgcGFyZW50Tm9kZSwgbmV3RWRnZSwgc291cmNlTm9kZSwgdGFyZ2V0Tm9kZSlcbntcbiAgLy90aGVyZSBhcmUganVzdCAyIHBhcmFtZXRlcnMgYXJlIHBhc3NlZCB0aGVuIGl0IGFkZHMgYW4gTEdyYXBoIGVsc2UgaXQgYWRkcyBhbiBMRWRnZVxuICBpZiAobmV3RWRnZSA9PSBudWxsICYmIHNvdXJjZU5vZGUgPT0gbnVsbCAmJiB0YXJnZXROb2RlID09IG51bGwpIHtcbiAgICBpZiAobmV3R3JhcGggPT0gbnVsbCkge1xuICAgICAgdGhyb3cgXCJHcmFwaCBpcyBudWxsIVwiO1xuICAgIH1cbiAgICBpZiAocGFyZW50Tm9kZSA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcIlBhcmVudCBub2RlIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmICh0aGlzLmdyYXBocy5pbmRleE9mKG5ld0dyYXBoKSA+IC0xKSB7XG4gICAgICB0aHJvdyBcIkdyYXBoIGFscmVhZHkgaW4gdGhpcyBncmFwaCBtZ3IhXCI7XG4gICAgfVxuXG4gICAgdGhpcy5ncmFwaHMucHVzaChuZXdHcmFwaCk7XG5cbiAgICBpZiAobmV3R3JhcGgucGFyZW50ICE9IG51bGwpIHtcbiAgICAgIHRocm93IFwiQWxyZWFkeSBoYXMgYSBwYXJlbnQhXCI7XG4gICAgfVxuICAgIGlmIChwYXJlbnROb2RlLmNoaWxkICE9IG51bGwpIHtcbiAgICAgIHRocm93ICBcIkFscmVhZHkgaGFzIGEgY2hpbGQhXCI7XG4gICAgfVxuXG4gICAgbmV3R3JhcGgucGFyZW50ID0gcGFyZW50Tm9kZTtcbiAgICBwYXJlbnROb2RlLmNoaWxkID0gbmV3R3JhcGg7XG5cbiAgICByZXR1cm4gbmV3R3JhcGg7XG4gIH1cbiAgZWxzZSB7XG4gICAgLy9jaGFuZ2UgdGhlIG9yZGVyIG9mIHRoZSBwYXJhbWV0ZXJzXG4gICAgdGFyZ2V0Tm9kZSA9IG5ld0VkZ2U7XG4gICAgc291cmNlTm9kZSA9IHBhcmVudE5vZGU7XG4gICAgbmV3RWRnZSA9IG5ld0dyYXBoO1xuICAgIHZhciBzb3VyY2VHcmFwaCA9IHNvdXJjZU5vZGUuZ2V0T3duZXIoKTtcbiAgICB2YXIgdGFyZ2V0R3JhcGggPSB0YXJnZXROb2RlLmdldE93bmVyKCk7XG5cbiAgICBpZiAoIShzb3VyY2VHcmFwaCAhPSBudWxsICYmIHNvdXJjZUdyYXBoLmdldEdyYXBoTWFuYWdlcigpID09IHRoaXMpKSB7XG4gICAgICB0aHJvdyBcIlNvdXJjZSBub3QgaW4gdGhpcyBncmFwaCBtZ3IhXCI7XG4gICAgfVxuICAgIGlmICghKHRhcmdldEdyYXBoICE9IG51bGwgJiYgdGFyZ2V0R3JhcGguZ2V0R3JhcGhNYW5hZ2VyKCkgPT0gdGhpcykpIHtcbiAgICAgIHRocm93IFwiVGFyZ2V0IG5vdCBpbiB0aGlzIGdyYXBoIG1nciFcIjtcbiAgICB9XG5cbiAgICBpZiAoc291cmNlR3JhcGggPT0gdGFyZ2V0R3JhcGgpXG4gICAge1xuICAgICAgbmV3RWRnZS5pc0ludGVyR3JhcGggPSBmYWxzZTtcbiAgICAgIHJldHVybiBzb3VyY2VHcmFwaC5hZGQobmV3RWRnZSwgc291cmNlTm9kZSwgdGFyZ2V0Tm9kZSk7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICBuZXdFZGdlLmlzSW50ZXJHcmFwaCA9IHRydWU7XG5cbiAgICAgIC8vIHNldCBzb3VyY2UgYW5kIHRhcmdldFxuICAgICAgbmV3RWRnZS5zb3VyY2UgPSBzb3VyY2VOb2RlO1xuICAgICAgbmV3RWRnZS50YXJnZXQgPSB0YXJnZXROb2RlO1xuXG4gICAgICAvLyBhZGQgZWRnZSB0byBpbnRlci1ncmFwaCBlZGdlIGxpc3RcbiAgICAgIGlmICh0aGlzLmVkZ2VzLmluZGV4T2YobmV3RWRnZSkgPiAtMSkge1xuICAgICAgICB0aHJvdyBcIkVkZ2UgYWxyZWFkeSBpbiBpbnRlci1ncmFwaCBlZGdlIGxpc3QhXCI7XG4gICAgICB9XG5cbiAgICAgIHRoaXMuZWRnZXMucHVzaChuZXdFZGdlKTtcblxuICAgICAgLy8gYWRkIGVkZ2UgdG8gc291cmNlIGFuZCB0YXJnZXQgaW5jaWRlbmN5IGxpc3RzXG4gICAgICBpZiAoIShuZXdFZGdlLnNvdXJjZSAhPSBudWxsICYmIG5ld0VkZ2UudGFyZ2V0ICE9IG51bGwpKSB7XG4gICAgICAgIHRocm93IFwiRWRnZSBzb3VyY2UgYW5kL29yIHRhcmdldCBpcyBudWxsIVwiO1xuICAgICAgfVxuXG4gICAgICBpZiAoIShuZXdFZGdlLnNvdXJjZS5lZGdlcy5pbmRleE9mKG5ld0VkZ2UpID09IC0xICYmIG5ld0VkZ2UudGFyZ2V0LmVkZ2VzLmluZGV4T2YobmV3RWRnZSkgPT0gLTEpKSB7XG4gICAgICAgIHRocm93IFwiRWRnZSBhbHJlYWR5IGluIHNvdXJjZSBhbmQvb3IgdGFyZ2V0IGluY2lkZW5jeSBsaXN0IVwiO1xuICAgICAgfVxuXG4gICAgICBuZXdFZGdlLnNvdXJjZS5lZGdlcy5wdXNoKG5ld0VkZ2UpO1xuICAgICAgbmV3RWRnZS50YXJnZXQuZWRnZXMucHVzaChuZXdFZGdlKTtcblxuICAgICAgcmV0dXJuIG5ld0VkZ2U7XG4gICAgfVxuICB9XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5yZW1vdmUgPSBmdW5jdGlvbiAobE9iaikge1xuICBpZiAobE9iaiBpbnN0YW5jZW9mIExHcmFwaCkge1xuICAgIHZhciBncmFwaCA9IGxPYmo7XG4gICAgaWYgKGdyYXBoLmdldEdyYXBoTWFuYWdlcigpICE9IHRoaXMpIHtcbiAgICAgIHRocm93IFwiR3JhcGggbm90IGluIHRoaXMgZ3JhcGggbWdyXCI7XG4gICAgfVxuICAgIGlmICghKGdyYXBoID09IHRoaXMucm9vdEdyYXBoIHx8IChncmFwaC5wYXJlbnQgIT0gbnVsbCAmJiBncmFwaC5wYXJlbnQuZ3JhcGhNYW5hZ2VyID09IHRoaXMpKSkge1xuICAgICAgdGhyb3cgXCJJbnZhbGlkIHBhcmVudCBub2RlIVwiO1xuICAgIH1cblxuICAgIC8vIGZpcnN0IHRoZSBlZGdlcyAobWFrZSBhIGNvcHkgdG8gZG8gaXQgc2FmZWx5KVxuICAgIHZhciBlZGdlc1RvQmVSZW1vdmVkID0gW107XG5cbiAgICBlZGdlc1RvQmVSZW1vdmVkID0gZWRnZXNUb0JlUmVtb3ZlZC5jb25jYXQoZ3JhcGguZ2V0RWRnZXMoKSk7XG5cbiAgICB2YXIgZWRnZTtcbiAgICB2YXIgcyA9IGVkZ2VzVG9CZVJlbW92ZWQubGVuZ3RoO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICAgIHtcbiAgICAgIGVkZ2UgPSBlZGdlc1RvQmVSZW1vdmVkW2ldO1xuICAgICAgZ3JhcGgucmVtb3ZlKGVkZ2UpO1xuICAgIH1cblxuICAgIC8vIHRoZW4gdGhlIG5vZGVzIChtYWtlIGEgY29weSB0byBkbyBpdCBzYWZlbHkpXG4gICAgdmFyIG5vZGVzVG9CZVJlbW92ZWQgPSBbXTtcblxuICAgIG5vZGVzVG9CZVJlbW92ZWQgPSBub2Rlc1RvQmVSZW1vdmVkLmNvbmNhdChncmFwaC5nZXROb2RlcygpKTtcblxuICAgIHZhciBub2RlO1xuICAgIHMgPSBub2Rlc1RvQmVSZW1vdmVkLmxlbmd0aDtcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAgICB7XG4gICAgICBub2RlID0gbm9kZXNUb0JlUmVtb3ZlZFtpXTtcbiAgICAgIGdyYXBoLnJlbW92ZShub2RlKTtcbiAgICB9XG5cbiAgICAvLyBjaGVjayBpZiBncmFwaCBpcyB0aGUgcm9vdFxuICAgIGlmIChncmFwaCA9PSB0aGlzLnJvb3RHcmFwaClcbiAgICB7XG4gICAgICB0aGlzLnNldFJvb3RHcmFwaChudWxsKTtcbiAgICB9XG5cbiAgICAvLyBub3cgcmVtb3ZlIHRoZSBncmFwaCBpdHNlbGZcbiAgICB2YXIgaW5kZXggPSB0aGlzLmdyYXBocy5pbmRleE9mKGdyYXBoKTtcbiAgICB0aGlzLmdyYXBocy5zcGxpY2UoaW5kZXgsIDEpO1xuXG4gICAgLy8gYWxzbyByZXNldCB0aGUgcGFyZW50IG9mIHRoZSBncmFwaFxuICAgIGdyYXBoLnBhcmVudCA9IG51bGw7XG4gIH1cbiAgZWxzZSBpZiAobE9iaiBpbnN0YW5jZW9mIExFZGdlKSB7XG4gICAgZWRnZSA9IGxPYmo7XG4gICAgaWYgKGVkZ2UgPT0gbnVsbCkge1xuICAgICAgdGhyb3cgXCJFZGdlIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmICghZWRnZS5pc0ludGVyR3JhcGgpIHtcbiAgICAgIHRocm93IFwiTm90IGFuIGludGVyLWdyYXBoIGVkZ2UhXCI7XG4gICAgfVxuICAgIGlmICghKGVkZ2Uuc291cmNlICE9IG51bGwgJiYgZWRnZS50YXJnZXQgIT0gbnVsbCkpIHtcbiAgICAgIHRocm93IFwiU291cmNlIGFuZC9vciB0YXJnZXQgaXMgbnVsbCFcIjtcbiAgICB9XG5cbiAgICAvLyByZW1vdmUgZWRnZSBmcm9tIHNvdXJjZSBhbmQgdGFyZ2V0IG5vZGVzJyBpbmNpZGVuY3kgbGlzdHNcblxuICAgIGlmICghKGVkZ2Uuc291cmNlLmVkZ2VzLmluZGV4T2YoZWRnZSkgIT0gLTEgJiYgZWRnZS50YXJnZXQuZWRnZXMuaW5kZXhPZihlZGdlKSAhPSAtMSkpIHtcbiAgICAgIHRocm93IFwiU291cmNlIGFuZC9vciB0YXJnZXQgZG9lc24ndCBrbm93IHRoaXMgZWRnZSFcIjtcbiAgICB9XG5cbiAgICB2YXIgaW5kZXggPSBlZGdlLnNvdXJjZS5lZGdlcy5pbmRleE9mKGVkZ2UpO1xuICAgIGVkZ2Uuc291cmNlLmVkZ2VzLnNwbGljZShpbmRleCwgMSk7XG4gICAgaW5kZXggPSBlZGdlLnRhcmdldC5lZGdlcy5pbmRleE9mKGVkZ2UpO1xuICAgIGVkZ2UudGFyZ2V0LmVkZ2VzLnNwbGljZShpbmRleCwgMSk7XG5cbiAgICAvLyByZW1vdmUgZWRnZSBmcm9tIG93bmVyIGdyYXBoIG1hbmFnZXIncyBpbnRlci1ncmFwaCBlZGdlIGxpc3RcblxuICAgIGlmICghKGVkZ2Uuc291cmNlLm93bmVyICE9IG51bGwgJiYgZWRnZS5zb3VyY2Uub3duZXIuZ2V0R3JhcGhNYW5hZ2VyKCkgIT0gbnVsbCkpIHtcbiAgICAgIHRocm93IFwiRWRnZSBvd25lciBncmFwaCBvciBvd25lciBncmFwaCBtYW5hZ2VyIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmIChlZGdlLnNvdXJjZS5vd25lci5nZXRHcmFwaE1hbmFnZXIoKS5lZGdlcy5pbmRleE9mKGVkZ2UpID09IC0xKSB7XG4gICAgICB0aHJvdyBcIk5vdCBpbiBvd25lciBncmFwaCBtYW5hZ2VyJ3MgZWRnZSBsaXN0IVwiO1xuICAgIH1cblxuICAgIHZhciBpbmRleCA9IGVkZ2Uuc291cmNlLm93bmVyLmdldEdyYXBoTWFuYWdlcigpLmVkZ2VzLmluZGV4T2YoZWRnZSk7XG4gICAgZWRnZS5zb3VyY2Uub3duZXIuZ2V0R3JhcGhNYW5hZ2VyKCkuZWRnZXMuc3BsaWNlKGluZGV4LCAxKTtcbiAgfVxufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUudXBkYXRlQm91bmRzID0gZnVuY3Rpb24gKClcbntcbiAgdGhpcy5yb290R3JhcGgudXBkYXRlQm91bmRzKHRydWUpO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0R3JhcGhzID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuZ3JhcGhzO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0QWxsTm9kZXMgPSBmdW5jdGlvbiAoKVxue1xuICBpZiAodGhpcy5hbGxOb2RlcyA9PSBudWxsKVxuICB7XG4gICAgdmFyIG5vZGVMaXN0ID0gW107XG4gICAgdmFyIGdyYXBocyA9IHRoaXMuZ2V0R3JhcGhzKCk7XG4gICAgdmFyIHMgPSBncmFwaHMubGVuZ3RoO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICAgIHtcbiAgICAgIG5vZGVMaXN0ID0gbm9kZUxpc3QuY29uY2F0KGdyYXBoc1tpXS5nZXROb2RlcygpKTtcbiAgICB9XG4gICAgdGhpcy5hbGxOb2RlcyA9IG5vZGVMaXN0O1xuICB9XG4gIHJldHVybiB0aGlzLmFsbE5vZGVzO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUucmVzZXRBbGxOb2RlcyA9IGZ1bmN0aW9uICgpXG57XG4gIHRoaXMuYWxsTm9kZXMgPSBudWxsO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUucmVzZXRBbGxFZGdlcyA9IGZ1bmN0aW9uICgpXG57XG4gIHRoaXMuYWxsRWRnZXMgPSBudWxsO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbiA9IGZ1bmN0aW9uICgpXG57XG4gIHRoaXMuYWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24gPSBudWxsO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0QWxsRWRnZXMgPSBmdW5jdGlvbiAoKVxue1xuICBpZiAodGhpcy5hbGxFZGdlcyA9PSBudWxsKVxuICB7XG4gICAgdmFyIGVkZ2VMaXN0ID0gW107XG4gICAgdmFyIGdyYXBocyA9IHRoaXMuZ2V0R3JhcGhzKCk7XG4gICAgdmFyIHMgPSBncmFwaHMubGVuZ3RoO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZ3JhcGhzLmxlbmd0aDsgaSsrKVxuICAgIHtcbiAgICAgIGVkZ2VMaXN0ID0gZWRnZUxpc3QuY29uY2F0KGdyYXBoc1tpXS5nZXRFZGdlcygpKTtcbiAgICB9XG5cbiAgICBlZGdlTGlzdCA9IGVkZ2VMaXN0LmNvbmNhdCh0aGlzLmVkZ2VzKTtcblxuICAgIHRoaXMuYWxsRWRnZXMgPSBlZGdlTGlzdDtcbiAgfVxuICByZXR1cm4gdGhpcy5hbGxFZGdlcztcbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmdldEFsbE5vZGVzVG9BcHBseUdyYXZpdGF0aW9uID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuYWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb247XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5zZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbiA9IGZ1bmN0aW9uIChub2RlTGlzdClcbntcbiAgaWYgKHRoaXMuYWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24gIT0gbnVsbCkge1xuICAgIHRocm93IFwiYXNzZXJ0IGZhaWxlZFwiO1xuICB9XG5cbiAgdGhpcy5hbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbiA9IG5vZGVMaXN0O1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0Um9vdCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnJvb3RHcmFwaDtcbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLnNldFJvb3RHcmFwaCA9IGZ1bmN0aW9uIChncmFwaClcbntcbiAgaWYgKGdyYXBoLmdldEdyYXBoTWFuYWdlcigpICE9IHRoaXMpIHtcbiAgICB0aHJvdyBcIlJvb3Qgbm90IGluIHRoaXMgZ3JhcGggbWdyIVwiO1xuICB9XG5cbiAgdGhpcy5yb290R3JhcGggPSBncmFwaDtcbiAgLy8gcm9vdCBncmFwaCBtdXN0IGhhdmUgYSByb290IG5vZGUgYXNzb2NpYXRlZCB3aXRoIGl0IGZvciBjb252ZW5pZW5jZVxuICBpZiAoZ3JhcGgucGFyZW50ID09IG51bGwpXG4gIHtcbiAgICBncmFwaC5wYXJlbnQgPSB0aGlzLmxheW91dC5uZXdOb2RlKFwiUm9vdCBub2RlXCIpO1xuICB9XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5nZXRMYXlvdXQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sYXlvdXQ7XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5pc09uZUFuY2VzdG9yT2ZPdGhlciA9IGZ1bmN0aW9uIChmaXJzdE5vZGUsIHNlY29uZE5vZGUpXG57XG4gIGlmICghKGZpcnN0Tm9kZSAhPSBudWxsICYmIHNlY29uZE5vZGUgIT0gbnVsbCkpIHtcbiAgICB0aHJvdyBcImFzc2VydCBmYWlsZWRcIjtcbiAgfVxuXG4gIGlmIChmaXJzdE5vZGUgPT0gc2Vjb25kTm9kZSlcbiAge1xuICAgIHJldHVybiB0cnVlO1xuICB9XG4gIC8vIElzIHNlY29uZCBub2RlIGFuIGFuY2VzdG9yIG9mIHRoZSBmaXJzdCBvbmU/XG4gIHZhciBvd25lckdyYXBoID0gZmlyc3ROb2RlLmdldE93bmVyKCk7XG4gIHZhciBwYXJlbnROb2RlO1xuXG4gIGRvXG4gIHtcbiAgICBwYXJlbnROb2RlID0gb3duZXJHcmFwaC5nZXRQYXJlbnQoKTtcblxuICAgIGlmIChwYXJlbnROb2RlID09IG51bGwpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuXG4gICAgaWYgKHBhcmVudE5vZGUgPT0gc2Vjb25kTm9kZSlcbiAgICB7XG4gICAgICByZXR1cm4gdHJ1ZTtcbiAgICB9XG5cbiAgICBvd25lckdyYXBoID0gcGFyZW50Tm9kZS5nZXRPd25lcigpO1xuICAgIGlmIChvd25lckdyYXBoID09IG51bGwpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9IHdoaWxlICh0cnVlKTtcbiAgLy8gSXMgZmlyc3Qgbm9kZSBhbiBhbmNlc3RvciBvZiB0aGUgc2Vjb25kIG9uZT9cbiAgb3duZXJHcmFwaCA9IHNlY29uZE5vZGUuZ2V0T3duZXIoKTtcblxuICBkb1xuICB7XG4gICAgcGFyZW50Tm9kZSA9IG93bmVyR3JhcGguZ2V0UGFyZW50KCk7XG5cbiAgICBpZiAocGFyZW50Tm9kZSA9PSBudWxsKVxuICAgIHtcbiAgICAgIGJyZWFrO1xuICAgIH1cblxuICAgIGlmIChwYXJlbnROb2RlID09IGZpcnN0Tm9kZSlcbiAgICB7XG4gICAgICByZXR1cm4gdHJ1ZTtcbiAgICB9XG5cbiAgICBvd25lckdyYXBoID0gcGFyZW50Tm9kZS5nZXRPd25lcigpO1xuICAgIGlmIChvd25lckdyYXBoID09IG51bGwpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9IHdoaWxlICh0cnVlKTtcblxuICByZXR1cm4gZmFsc2U7XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5jYWxjTG93ZXN0Q29tbW9uQW5jZXN0b3JzID0gZnVuY3Rpb24gKClcbntcbiAgdmFyIGVkZ2U7XG4gIHZhciBzb3VyY2VOb2RlO1xuICB2YXIgdGFyZ2V0Tm9kZTtcbiAgdmFyIHNvdXJjZUFuY2VzdG9yR3JhcGg7XG4gIHZhciB0YXJnZXRBbmNlc3RvckdyYXBoO1xuXG4gIHZhciBlZGdlcyA9IHRoaXMuZ2V0QWxsRWRnZXMoKTtcbiAgdmFyIHMgPSBlZGdlcy5sZW5ndGg7XG4gIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICB7XG4gICAgZWRnZSA9IGVkZ2VzW2ldO1xuXG4gICAgc291cmNlTm9kZSA9IGVkZ2Uuc291cmNlO1xuICAgIHRhcmdldE5vZGUgPSBlZGdlLnRhcmdldDtcbiAgICBlZGdlLmxjYSA9IG51bGw7XG4gICAgZWRnZS5zb3VyY2VJbkxjYSA9IHNvdXJjZU5vZGU7XG4gICAgZWRnZS50YXJnZXRJbkxjYSA9IHRhcmdldE5vZGU7XG5cbiAgICBpZiAoc291cmNlTm9kZSA9PSB0YXJnZXROb2RlKVxuICAgIHtcbiAgICAgIGVkZ2UubGNhID0gc291cmNlTm9kZS5nZXRPd25lcigpO1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgc291cmNlQW5jZXN0b3JHcmFwaCA9IHNvdXJjZU5vZGUuZ2V0T3duZXIoKTtcblxuICAgIHdoaWxlIChlZGdlLmxjYSA9PSBudWxsKVxuICAgIHtcbiAgICAgIHRhcmdldEFuY2VzdG9yR3JhcGggPSB0YXJnZXROb2RlLmdldE93bmVyKCk7XG5cbiAgICAgIHdoaWxlIChlZGdlLmxjYSA9PSBudWxsKVxuICAgICAge1xuICAgICAgICBpZiAodGFyZ2V0QW5jZXN0b3JHcmFwaCA9PSBzb3VyY2VBbmNlc3RvckdyYXBoKVxuICAgICAgICB7XG4gICAgICAgICAgZWRnZS5sY2EgPSB0YXJnZXRBbmNlc3RvckdyYXBoO1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICB9XG5cbiAgICAgICAgaWYgKHRhcmdldEFuY2VzdG9yR3JhcGggPT0gdGhpcy5yb290R3JhcGgpXG4gICAgICAgIHtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuXG4gICAgICAgIGlmIChlZGdlLmxjYSAhPSBudWxsKSB7XG4gICAgICAgICAgdGhyb3cgXCJhc3NlcnQgZmFpbGVkXCI7XG4gICAgICAgIH1cbiAgICAgICAgZWRnZS50YXJnZXRJbkxjYSA9IHRhcmdldEFuY2VzdG9yR3JhcGguZ2V0UGFyZW50KCk7XG4gICAgICAgIHRhcmdldEFuY2VzdG9yR3JhcGggPSBlZGdlLnRhcmdldEluTGNhLmdldE93bmVyKCk7XG4gICAgICB9XG5cbiAgICAgIGlmIChzb3VyY2VBbmNlc3RvckdyYXBoID09IHRoaXMucm9vdEdyYXBoKVxuICAgICAge1xuICAgICAgICBicmVhaztcbiAgICAgIH1cblxuICAgICAgaWYgKGVkZ2UubGNhID09IG51bGwpXG4gICAgICB7XG4gICAgICAgIGVkZ2Uuc291cmNlSW5MY2EgPSBzb3VyY2VBbmNlc3RvckdyYXBoLmdldFBhcmVudCgpO1xuICAgICAgICBzb3VyY2VBbmNlc3RvckdyYXBoID0gZWRnZS5zb3VyY2VJbkxjYS5nZXRPd25lcigpO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChlZGdlLmxjYSA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcImFzc2VydCBmYWlsZWRcIjtcbiAgICB9XG4gIH1cbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmNhbGNMb3dlc3RDb21tb25BbmNlc3RvciA9IGZ1bmN0aW9uIChmaXJzdE5vZGUsIHNlY29uZE5vZGUpXG57XG4gIGlmIChmaXJzdE5vZGUgPT0gc2Vjb25kTm9kZSlcbiAge1xuICAgIHJldHVybiBmaXJzdE5vZGUuZ2V0T3duZXIoKTtcbiAgfVxuICB2YXIgZmlyc3RPd25lckdyYXBoID0gZmlyc3ROb2RlLmdldE93bmVyKCk7XG5cbiAgZG9cbiAge1xuICAgIGlmIChmaXJzdE93bmVyR3JhcGggPT0gbnVsbClcbiAgICB7XG4gICAgICBicmVhaztcbiAgICB9XG4gICAgdmFyIHNlY29uZE93bmVyR3JhcGggPSBzZWNvbmROb2RlLmdldE93bmVyKCk7XG5cbiAgICBkb1xuICAgIHtcbiAgICAgIGlmIChzZWNvbmRPd25lckdyYXBoID09IG51bGwpXG4gICAgICB7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuXG4gICAgICBpZiAoc2Vjb25kT3duZXJHcmFwaCA9PSBmaXJzdE93bmVyR3JhcGgpXG4gICAgICB7XG4gICAgICAgIHJldHVybiBzZWNvbmRPd25lckdyYXBoO1xuICAgICAgfVxuICAgICAgc2Vjb25kT3duZXJHcmFwaCA9IHNlY29uZE93bmVyR3JhcGguZ2V0UGFyZW50KCkuZ2V0T3duZXIoKTtcbiAgICB9IHdoaWxlICh0cnVlKTtcblxuICAgIGZpcnN0T3duZXJHcmFwaCA9IGZpcnN0T3duZXJHcmFwaC5nZXRQYXJlbnQoKS5nZXRPd25lcigpO1xuICB9IHdoaWxlICh0cnVlKTtcblxuICByZXR1cm4gZmlyc3RPd25lckdyYXBoO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuY2FsY0luY2x1c2lvblRyZWVEZXB0aHMgPSBmdW5jdGlvbiAoZ3JhcGgsIGRlcHRoKSB7XG4gIGlmIChncmFwaCA9PSBudWxsICYmIGRlcHRoID09IG51bGwpIHtcbiAgICBncmFwaCA9IHRoaXMucm9vdEdyYXBoO1xuICAgIGRlcHRoID0gMTtcbiAgfVxuICB2YXIgbm9kZTtcblxuICB2YXIgbm9kZXMgPSBncmFwaC5nZXROb2RlcygpO1xuICB2YXIgcyA9IG5vZGVzLmxlbmd0aDtcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBzOyBpKyspXG4gIHtcbiAgICBub2RlID0gbm9kZXNbaV07XG4gICAgbm9kZS5pbmNsdXNpb25UcmVlRGVwdGggPSBkZXB0aDtcblxuICAgIGlmIChub2RlLmNoaWxkICE9IG51bGwpXG4gICAge1xuICAgICAgdGhpcy5jYWxjSW5jbHVzaW9uVHJlZURlcHRocyhub2RlLmNoaWxkLCBkZXB0aCArIDEpO1xuICAgIH1cbiAgfVxufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuaW5jbHVkZXNJbnZhbGlkRWRnZSA9IGZ1bmN0aW9uICgpXG57XG4gIHZhciBlZGdlO1xuXG4gIHZhciBzID0gdGhpcy5lZGdlcy5sZW5ndGg7XG4gIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICB7XG4gICAgZWRnZSA9IHRoaXMuZWRnZXNbaV07XG5cbiAgICBpZiAodGhpcy5pc09uZUFuY2VzdG9yT2ZPdGhlcihlZGdlLnNvdXJjZSwgZWRnZS50YXJnZXQpKVxuICAgIHtcbiAgICAgIHJldHVybiB0cnVlO1xuICAgIH1cbiAgfVxuICByZXR1cm4gZmFsc2U7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IExHcmFwaE1hbmFnZXI7XG4iLCJmdW5jdGlvbiBMR3JhcGhPYmplY3QodkdyYXBoT2JqZWN0KSB7XG4gIHRoaXMudkdyYXBoT2JqZWN0ID0gdkdyYXBoT2JqZWN0O1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IExHcmFwaE9iamVjdDtcbiIsInZhciBMR3JhcGhPYmplY3QgPSByZXF1aXJlKCcuL0xHcmFwaE9iamVjdCcpO1xudmFyIEludGVnZXIgPSByZXF1aXJlKCcuL0ludGVnZXInKTtcbnZhciBSZWN0YW5nbGVEID0gcmVxdWlyZSgnLi9SZWN0YW5nbGVEJyk7XG5cbmZ1bmN0aW9uIExOb2RlKGdtLCBsb2MsIHNpemUsIHZOb2RlKSB7XG4gIC8vQWx0ZXJuYXRpdmUgY29uc3RydWN0b3IgMSA6IExOb2RlKExHcmFwaE1hbmFnZXIgZ20sIFBvaW50IGxvYywgRGltZW5zaW9uIHNpemUsIE9iamVjdCB2Tm9kZSlcbiAgaWYgKHNpemUgPT0gbnVsbCAmJiB2Tm9kZSA9PSBudWxsKSB7XG4gICAgdk5vZGUgPSBsb2M7XG4gIH1cblxuICBMR3JhcGhPYmplY3QuY2FsbCh0aGlzLCB2Tm9kZSk7XG5cbiAgLy9BbHRlcm5hdGl2ZSBjb25zdHJ1Y3RvciAyIDogTE5vZGUoTGF5b3V0IGxheW91dCwgT2JqZWN0IHZOb2RlKVxuICBpZiAoZ20uZ3JhcGhNYW5hZ2VyICE9IG51bGwpXG4gICAgZ20gPSBnbS5ncmFwaE1hbmFnZXI7XG5cbiAgdGhpcy5lc3RpbWF0ZWRTaXplID0gSW50ZWdlci5NSU5fVkFMVUU7XG4gIHRoaXMuaW5jbHVzaW9uVHJlZURlcHRoID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gIHRoaXMudkdyYXBoT2JqZWN0ID0gdk5vZGU7XG4gIHRoaXMuZWRnZXMgPSBbXTtcbiAgdGhpcy5ncmFwaE1hbmFnZXIgPSBnbTtcblxuICBpZiAoc2l6ZSAhPSBudWxsICYmIGxvYyAhPSBudWxsKVxuICAgIHRoaXMucmVjdCA9IG5ldyBSZWN0YW5nbGVEKGxvYy54LCBsb2MueSwgc2l6ZS53aWR0aCwgc2l6ZS5oZWlnaHQpO1xuICBlbHNlXG4gICAgdGhpcy5yZWN0ID0gbmV3IFJlY3RhbmdsZUQoKTtcbn1cblxuTE5vZGUucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMR3JhcGhPYmplY3QucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gTEdyYXBoT2JqZWN0KSB7XG4gIExOb2RlW3Byb3BdID0gTEdyYXBoT2JqZWN0W3Byb3BdO1xufVxuXG5MTm9kZS5wcm90b3R5cGUuZ2V0RWRnZXMgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5lZGdlcztcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRDaGlsZCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmNoaWxkO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldE93bmVyID0gZnVuY3Rpb24gKClcbntcbiAgaWYgKHRoaXMub3duZXIgIT0gbnVsbCkge1xuICAgIGlmICghKHRoaXMub3duZXIgPT0gbnVsbCB8fCB0aGlzLm93bmVyLmdldE5vZGVzKCkuaW5kZXhPZih0aGlzKSA+IC0xKSkge1xuICAgICAgdGhyb3cgXCJhc3NlcnQgZmFpbGVkXCI7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIHRoaXMub3duZXI7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0V2lkdGggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LndpZHRoO1xufTtcblxuTE5vZGUucHJvdG90eXBlLnNldFdpZHRoID0gZnVuY3Rpb24gKHdpZHRoKVxue1xuICB0aGlzLnJlY3Qud2lkdGggPSB3aWR0aDtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRIZWlnaHQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LmhlaWdodDtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5zZXRIZWlnaHQgPSBmdW5jdGlvbiAoaGVpZ2h0KVxue1xuICB0aGlzLnJlY3QuaGVpZ2h0ID0gaGVpZ2h0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldENlbnRlclggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LnggKyB0aGlzLnJlY3Qud2lkdGggLyAyO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldENlbnRlclkgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LnkgKyB0aGlzLnJlY3QuaGVpZ2h0IC8gMjtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRDZW50ZXIgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLnJlY3QueCArIHRoaXMucmVjdC53aWR0aCAvIDIsXG4gICAgICAgICAgdGhpcy5yZWN0LnkgKyB0aGlzLnJlY3QuaGVpZ2h0IC8gMik7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0TG9jYXRpb24gPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLnJlY3QueCwgdGhpcy5yZWN0LnkpO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFJlY3QgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldERpYWdvbmFsID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIE1hdGguc3FydCh0aGlzLnJlY3Qud2lkdGggKiB0aGlzLnJlY3Qud2lkdGggK1xuICAgICAgICAgIHRoaXMucmVjdC5oZWlnaHQgKiB0aGlzLnJlY3QuaGVpZ2h0KTtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5zZXRSZWN0ID0gZnVuY3Rpb24gKHVwcGVyTGVmdCwgZGltZW5zaW9uKVxue1xuICB0aGlzLnJlY3QueCA9IHVwcGVyTGVmdC54O1xuICB0aGlzLnJlY3QueSA9IHVwcGVyTGVmdC55O1xuICB0aGlzLnJlY3Qud2lkdGggPSBkaW1lbnNpb24ud2lkdGg7XG4gIHRoaXMucmVjdC5oZWlnaHQgPSBkaW1lbnNpb24uaGVpZ2h0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLnNldENlbnRlciA9IGZ1bmN0aW9uIChjeCwgY3kpXG57XG4gIHRoaXMucmVjdC54ID0gY3ggLSB0aGlzLnJlY3Qud2lkdGggLyAyO1xuICB0aGlzLnJlY3QueSA9IGN5IC0gdGhpcy5yZWN0LmhlaWdodCAvIDI7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuc2V0TG9jYXRpb24gPSBmdW5jdGlvbiAoeCwgeSlcbntcbiAgdGhpcy5yZWN0LnggPSB4O1xuICB0aGlzLnJlY3QueSA9IHk7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUubW92ZUJ5ID0gZnVuY3Rpb24gKGR4LCBkeSlcbntcbiAgdGhpcy5yZWN0LnggKz0gZHg7XG4gIHRoaXMucmVjdC55ICs9IGR5O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldEVkZ2VMaXN0VG9Ob2RlID0gZnVuY3Rpb24gKHRvKVxue1xuICB2YXIgZWRnZUxpc3QgPSBbXTtcbiAgdmFyIGVkZ2U7XG5cbiAgZm9yICh2YXIgb2JqIGluIHRoaXMuZWRnZXMpXG4gIHtcbiAgICBlZGdlID0gb2JqO1xuXG4gICAgaWYgKGVkZ2UudGFyZ2V0ID09IHRvKVxuICAgIHtcbiAgICAgIGlmIChlZGdlLnNvdXJjZSAhPSB0aGlzKVxuICAgICAgICB0aHJvdyBcIkluY29ycmVjdCBlZGdlIHNvdXJjZSFcIjtcblxuICAgICAgZWRnZUxpc3QucHVzaChlZGdlKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gZWRnZUxpc3Q7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0RWRnZXNCZXR3ZWVuID0gZnVuY3Rpb24gKG90aGVyKVxue1xuICB2YXIgZWRnZUxpc3QgPSBbXTtcbiAgdmFyIGVkZ2U7XG5cbiAgZm9yICh2YXIgb2JqIGluIHRoaXMuZWRnZXMpXG4gIHtcbiAgICBlZGdlID0gdGhpcy5lZGdlc1tvYmpdO1xuXG4gICAgaWYgKCEoZWRnZS5zb3VyY2UgPT0gdGhpcyB8fCBlZGdlLnRhcmdldCA9PSB0aGlzKSlcbiAgICAgIHRocm93IFwiSW5jb3JyZWN0IGVkZ2Ugc291cmNlIGFuZC9vciB0YXJnZXRcIjtcblxuICAgIGlmICgoZWRnZS50YXJnZXQgPT0gb3RoZXIpIHx8IChlZGdlLnNvdXJjZSA9PSBvdGhlcikpXG4gICAge1xuICAgICAgZWRnZUxpc3QucHVzaChlZGdlKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gZWRnZUxpc3Q7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0TmVpZ2hib3JzTGlzdCA9IGZ1bmN0aW9uICgpXG57XG4gIHZhciBuZWlnaGJvcnMgPSBuZXcgSGFzaFNldCgpO1xuICB2YXIgZWRnZTtcblxuICBmb3IgKHZhciBvYmogaW4gdGhpcy5lZGdlcylcbiAge1xuICAgIGVkZ2UgPSB0aGlzLmVkZ2VzW29ial07XG5cbiAgICBpZiAoZWRnZS5zb3VyY2UgPT0gdGhpcylcbiAgICB7XG4gICAgICBuZWlnaGJvcnMuYWRkKGVkZ2UudGFyZ2V0KTtcbiAgICB9XG4gICAgZWxzZVxuICAgIHtcbiAgICAgIGlmICghZWRnZS50YXJnZXQgPT0gdGhpcylcbiAgICAgICAgdGhyb3cgXCJJbmNvcnJlY3QgaW5jaWRlbmN5IVwiO1xuICAgICAgbmVpZ2hib3JzLmFkZChlZGdlLnNvdXJjZSk7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIG5laWdoYm9ycztcbn07XG5cbkxOb2RlLnByb3RvdHlwZS53aXRoQ2hpbGRyZW4gPSBmdW5jdGlvbiAoKVxue1xuICB2YXIgd2l0aE5laWdoYm9yc0xpc3QgPSBbXTtcbiAgdmFyIGNoaWxkTm9kZTtcblxuICB3aXRoTmVpZ2hib3JzTGlzdC5wdXNoKHRoaXMpO1xuXG4gIGlmICh0aGlzLmNoaWxkICE9IG51bGwpXG4gIHtcbiAgICB2YXIgbm9kZXMgPSB0aGlzLmNoaWxkLmdldE5vZGVzKCk7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKylcbiAgICB7XG4gICAgICBjaGlsZE5vZGUgPSBub2Rlc1tpXTtcblxuICAgICAgd2l0aE5laWdoYm9yc0xpc3QgPSB3aXRoTmVpZ2hib3JzTGlzdC5jb25jYXQoY2hpbGROb2RlLndpdGhDaGlsZHJlbigpKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gd2l0aE5laWdoYm9yc0xpc3Q7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0RXN0aW1hdGVkU2l6ZSA9IGZ1bmN0aW9uICgpIHtcbiAgaWYgKHRoaXMuZXN0aW1hdGVkU2l6ZSA9PSBJbnRlZ2VyLk1JTl9WQUxVRSkge1xuICAgIHRocm93IFwiYXNzZXJ0IGZhaWxlZFwiO1xuICB9XG4gIHJldHVybiB0aGlzLmVzdGltYXRlZFNpemU7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuY2FsY0VzdGltYXRlZFNpemUgPSBmdW5jdGlvbiAoKSB7XG4gIGlmICh0aGlzLmNoaWxkID09IG51bGwpXG4gIHtcbiAgICByZXR1cm4gdGhpcy5lc3RpbWF0ZWRTaXplID0gTWF0aC5mbG9vcigodGhpcy5yZWN0LndpZHRoICsgdGhpcy5yZWN0LmhlaWdodCkgLyAyKTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICB0aGlzLmVzdGltYXRlZFNpemUgPSB0aGlzLmNoaWxkLmNhbGNFc3RpbWF0ZWRTaXplKCk7XG4gICAgdGhpcy5yZWN0LndpZHRoID0gdGhpcy5lc3RpbWF0ZWRTaXplO1xuICAgIHRoaXMucmVjdC5oZWlnaHQgPSB0aGlzLmVzdGltYXRlZFNpemU7XG5cbiAgICByZXR1cm4gdGhpcy5lc3RpbWF0ZWRTaXplO1xuICB9XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuc2NhdHRlciA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIHJhbmRvbUNlbnRlclg7XG4gIHZhciByYW5kb21DZW50ZXJZO1xuXG4gIHZhciBtaW5YID0gLUxheW91dENvbnN0YW50cy5JTklUSUFMX1dPUkxEX0JPVU5EQVJZO1xuICB2YXIgbWF4WCA9IExheW91dENvbnN0YW50cy5JTklUSUFMX1dPUkxEX0JPVU5EQVJZO1xuICByYW5kb21DZW50ZXJYID0gTGF5b3V0Q29uc3RhbnRzLldPUkxEX0NFTlRFUl9YICtcbiAgICAgICAgICAoUmFuZG9tU2VlZC5uZXh0RG91YmxlKCkgKiAobWF4WCAtIG1pblgpKSArIG1pblg7XG5cbiAgdmFyIG1pblkgPSAtTGF5b3V0Q29uc3RhbnRzLklOSVRJQUxfV09STERfQk9VTkRBUlk7XG4gIHZhciBtYXhZID0gTGF5b3V0Q29uc3RhbnRzLklOSVRJQUxfV09STERfQk9VTkRBUlk7XG4gIHJhbmRvbUNlbnRlclkgPSBMYXlvdXRDb25zdGFudHMuV09STERfQ0VOVEVSX1kgK1xuICAgICAgICAgIChSYW5kb21TZWVkLm5leHREb3VibGUoKSAqIChtYXhZIC0gbWluWSkpICsgbWluWTtcblxuICB0aGlzLnJlY3QueCA9IHJhbmRvbUNlbnRlclg7XG4gIHRoaXMucmVjdC55ID0gcmFuZG9tQ2VudGVyWVxufTtcblxuTE5vZGUucHJvdG90eXBlLnVwZGF0ZUJvdW5kcyA9IGZ1bmN0aW9uICgpIHtcbiAgaWYgKHRoaXMuZ2V0Q2hpbGQoKSA9PSBudWxsKSB7XG4gICAgdGhyb3cgXCJhc3NlcnQgZmFpbGVkXCI7XG4gIH1cbiAgaWYgKHRoaXMuZ2V0Q2hpbGQoKS5nZXROb2RlcygpLmxlbmd0aCAhPSAwKVxuICB7XG4gICAgLy8gd3JhcCB0aGUgY2hpbGRyZW4gbm9kZXMgYnkgcmUtYXJyYW5naW5nIHRoZSBib3VuZGFyaWVzXG4gICAgdmFyIGNoaWxkR3JhcGggPSB0aGlzLmdldENoaWxkKCk7XG4gICAgY2hpbGRHcmFwaC51cGRhdGVCb3VuZHModHJ1ZSk7XG5cbiAgICB0aGlzLnJlY3QueCA9IGNoaWxkR3JhcGguZ2V0TGVmdCgpO1xuICAgIHRoaXMucmVjdC55ID0gY2hpbGRHcmFwaC5nZXRUb3AoKTtcblxuICAgIHRoaXMuc2V0V2lkdGgoY2hpbGRHcmFwaC5nZXRSaWdodCgpIC0gY2hpbGRHcmFwaC5nZXRMZWZ0KCkgK1xuICAgICAgICAgICAgMiAqIExheW91dENvbnN0YW50cy5DT01QT1VORF9OT0RFX01BUkdJTik7XG4gICAgdGhpcy5zZXRIZWlnaHQoY2hpbGRHcmFwaC5nZXRCb3R0b20oKSAtIGNoaWxkR3JhcGguZ2V0VG9wKCkgK1xuICAgICAgICAgICAgMiAqIExheW91dENvbnN0YW50cy5DT01QT1VORF9OT0RFX01BUkdJTiArXG4gICAgICAgICAgICBMYXlvdXRDb25zdGFudHMuTEFCRUxfSEVJR0hUKTtcbiAgfVxufTtcblxuTE5vZGUucHJvdG90eXBlLmdldEluY2x1c2lvblRyZWVEZXB0aCA9IGZ1bmN0aW9uICgpXG57XG4gIGlmICh0aGlzLmluY2x1c2lvblRyZWVEZXB0aCA9PSBJbnRlZ2VyLk1BWF9WQUxVRSkge1xuICAgIHRocm93IFwiYXNzZXJ0IGZhaWxlZFwiO1xuICB9XG4gIHJldHVybiB0aGlzLmluY2x1c2lvblRyZWVEZXB0aDtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS50cmFuc2Zvcm0gPSBmdW5jdGlvbiAodHJhbnMpXG57XG4gIHZhciBsZWZ0ID0gdGhpcy5yZWN0Lng7XG5cbiAgaWYgKGxlZnQgPiBMYXlvdXRDb25zdGFudHMuV09STERfQk9VTkRBUlkpXG4gIHtcbiAgICBsZWZ0ID0gTGF5b3V0Q29uc3RhbnRzLldPUkxEX0JPVU5EQVJZO1xuICB9XG4gIGVsc2UgaWYgKGxlZnQgPCAtTGF5b3V0Q29uc3RhbnRzLldPUkxEX0JPVU5EQVJZKVxuICB7XG4gICAgbGVmdCA9IC1MYXlvdXRDb25zdGFudHMuV09STERfQk9VTkRBUlk7XG4gIH1cblxuICB2YXIgdG9wID0gdGhpcy5yZWN0Lnk7XG5cbiAgaWYgKHRvcCA+IExheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWSlcbiAge1xuICAgIHRvcCA9IExheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWTtcbiAgfVxuICBlbHNlIGlmICh0b3AgPCAtTGF5b3V0Q29uc3RhbnRzLldPUkxEX0JPVU5EQVJZKVxuICB7XG4gICAgdG9wID0gLUxheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWTtcbiAgfVxuXG4gIHZhciBsZWZ0VG9wID0gbmV3IFBvaW50RChsZWZ0LCB0b3ApO1xuICB2YXIgdkxlZnRUb3AgPSB0cmFucy5pbnZlcnNlVHJhbnNmb3JtUG9pbnQobGVmdFRvcCk7XG5cbiAgdGhpcy5zZXRMb2NhdGlvbih2TGVmdFRvcC54LCB2TGVmdFRvcC55KTtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRMZWZ0ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMucmVjdC54O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFJpZ2h0ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMucmVjdC54ICsgdGhpcy5yZWN0LndpZHRoO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFRvcCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnJlY3QueTtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRCb3R0b20gPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LnkgKyB0aGlzLnJlY3QuaGVpZ2h0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFBhcmVudCA9IGZ1bmN0aW9uICgpXG57XG4gIGlmICh0aGlzLm93bmVyID09IG51bGwpXG4gIHtcbiAgICByZXR1cm4gbnVsbDtcbiAgfVxuXG4gIHJldHVybiB0aGlzLm93bmVyLmdldFBhcmVudCgpO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSBMTm9kZTtcbiIsInZhciBMYXlvdXRDb25zdGFudHMgPSByZXF1aXJlKCcuL0xheW91dENvbnN0YW50cycpO1xudmFyIEhhc2hNYXAgPSByZXF1aXJlKCcuL0hhc2hNYXAnKTtcbnZhciBMR3JhcGhNYW5hZ2VyID0gcmVxdWlyZSgnLi9MR3JhcGhNYW5hZ2VyJyk7XG5cbmZ1bmN0aW9uIExheW91dChpc1JlbW90ZVVzZSkge1xuICAvL0xheW91dCBRdWFsaXR5OiAwOnByb29mLCAxOmRlZmF1bHQsIDI6ZHJhZnRcbiAgdGhpcy5sYXlvdXRRdWFsaXR5ID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfUVVBTElUWTtcbiAgLy9XaGV0aGVyIGxheW91dCBzaG91bGQgY3JlYXRlIGJlbmRwb2ludHMgYXMgbmVlZGVkIG9yIG5vdFxuICB0aGlzLmNyZWF0ZUJlbmRzQXNOZWVkZWQgPVxuICAgICAgICAgIExheW91dENvbnN0YW50cy5ERUZBVUxUX0NSRUFURV9CRU5EU19BU19ORUVERUQ7XG4gIC8vV2hldGhlciBsYXlvdXQgc2hvdWxkIGJlIGluY3JlbWVudGFsIG9yIG5vdFxuICB0aGlzLmluY3JlbWVudGFsID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfSU5DUkVNRU5UQUw7XG4gIC8vV2hldGhlciB3ZSBhbmltYXRlIGZyb20gYmVmb3JlIHRvIGFmdGVyIGxheW91dCBub2RlIHBvc2l0aW9uc1xuICB0aGlzLmFuaW1hdGlvbk9uTGF5b3V0ID1cbiAgICAgICAgICBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fT05fTEFZT1VUO1xuICAvL1doZXRoZXIgd2UgYW5pbWF0ZSB0aGUgbGF5b3V0IHByb2Nlc3Mgb3Igbm90XG4gIHRoaXMuYW5pbWF0aW9uRHVyaW5nTGF5b3V0ID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX0RVUklOR19MQVlPVVQ7XG4gIC8vTnVtYmVyIGl0ZXJhdGlvbnMgdGhhdCBzaG91bGQgYmUgZG9uZSBiZXR3ZWVuIHR3byBzdWNjZXNzaXZlIGFuaW1hdGlvbnNcbiAgdGhpcy5hbmltYXRpb25QZXJpb2QgPSBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fUEVSSU9EO1xuICAvKipcbiAgICogV2hldGhlciBvciBub3QgbGVhZiBub2RlcyAobm9uLWNvbXBvdW5kIG5vZGVzKSBhcmUgb2YgdW5pZm9ybSBzaXplcy4gV2hlblxuICAgKiB0aGV5IGFyZSwgYm90aCBzcHJpbmcgYW5kIHJlcHVsc2lvbiBmb3JjZXMgYmV0d2VlbiB0d28gbGVhZiBub2RlcyBjYW4gYmVcbiAgICogY2FsY3VsYXRlZCB3aXRob3V0IHRoZSBleHBlbnNpdmUgY2xpcHBpbmcgcG9pbnQgY2FsY3VsYXRpb25zLCByZXN1bHRpbmdcbiAgICogaW4gbWFqb3Igc3BlZWQtdXAuXG4gICAqL1xuICB0aGlzLnVuaWZvcm1MZWFmTm9kZVNpemVzID1cbiAgICAgICAgICBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9VTklGT1JNX0xFQUZfTk9ERV9TSVpFUztcbiAgLyoqXG4gICAqIFRoaXMgaXMgdXNlZCBmb3IgY3JlYXRpb24gb2YgYmVuZHBvaW50cyBieSB1c2luZyBkdW1teSBub2RlcyBhbmQgZWRnZXMuXG4gICAqIE1hcHMgYW4gTEVkZ2UgdG8gaXRzIGR1bW15IGJlbmRwb2ludCBwYXRoLlxuICAgKi9cbiAgdGhpcy5lZGdlVG9EdW1teU5vZGVzID0gbmV3IEhhc2hNYXAoKTtcbiAgdGhpcy5ncmFwaE1hbmFnZXIgPSBuZXcgTEdyYXBoTWFuYWdlcih0aGlzKTtcbiAgdGhpcy5pc0xheW91dEZpbmlzaGVkID0gZmFsc2U7XG4gIHRoaXMuaXNTdWJMYXlvdXQgPSBmYWxzZTtcbiAgdGhpcy5pc1JlbW90ZVVzZSA9IGZhbHNlO1xuXG4gIGlmIChpc1JlbW90ZVVzZSAhPSBudWxsKSB7XG4gICAgdGhpcy5pc1JlbW90ZVVzZSA9IGlzUmVtb3RlVXNlO1xuICB9XG59XG5cbkxheW91dC5SQU5ET01fU0VFRCA9IDE7XG5cbkxheW91dC5wcm90b3R5cGUuZ2V0R3JhcGhNYW5hZ2VyID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gdGhpcy5ncmFwaE1hbmFnZXI7XG59O1xuXG5MYXlvdXQucHJvdG90eXBlLmdldEFsbE5vZGVzID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsTm9kZXMoKTtcbn07XG5cbkxheW91dC5wcm90b3R5cGUuZ2V0QWxsRWRnZXMgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiB0aGlzLmdyYXBoTWFuYWdlci5nZXRBbGxFZGdlcygpO1xufTtcblxuTGF5b3V0LnByb3RvdHlwZS5nZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbiA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEFsbE5vZGVzVG9BcHBseUdyYXZpdGF0aW9uKCk7XG59O1xuXG5MYXlvdXQucHJvdG90eXBlLm5ld0dyYXBoTWFuYWdlciA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGdtID0gbmV3IExHcmFwaE1hbmFnZXIodGhpcyk7XG4gIHRoaXMuZ3JhcGhNYW5hZ2VyID0gZ207XG4gIHJldHVybiBnbTtcbn07XG5cbkxheW91dC5wcm90b3R5cGUubmV3R3JhcGggPSBmdW5jdGlvbiAodkdyYXBoKVxue1xuICByZXR1cm4gbmV3IExHcmFwaChudWxsLCB0aGlzLmdyYXBoTWFuYWdlciwgdkdyYXBoKTtcbn07XG5cbkxheW91dC5wcm90b3R5cGUubmV3Tm9kZSA9IGZ1bmN0aW9uICh2Tm9kZSlcbntcbiAgcmV0dXJuIG5ldyBMTm9kZSh0aGlzLmdyYXBoTWFuYWdlciwgdk5vZGUpO1xufTtcblxuTGF5b3V0LnByb3RvdHlwZS5uZXdFZGdlID0gZnVuY3Rpb24gKHZFZGdlKVxue1xuICByZXR1cm4gbmV3IExFZGdlKG51bGwsIG51bGwsIHZFZGdlKTtcbn07XG5cbkxheW91dC5wcm90b3R5cGUucnVuTGF5b3V0ID0gZnVuY3Rpb24gKClcbntcbiAgdGhpcy5pc0xheW91dEZpbmlzaGVkID0gZmFsc2U7XG5cbiAgdGhpcy5pbml0UGFyYW1ldGVycygpO1xuICB2YXIgaXNMYXlvdXRTdWNjZXNzZnVsbDtcblxuICBpZiAoKHRoaXMuZ3JhcGhNYW5hZ2VyLmdldFJvb3QoKSA9PSBudWxsKVxuICAgICAgICAgIHx8IHRoaXMuZ3JhcGhNYW5hZ2VyLmdldFJvb3QoKS5nZXROb2RlcygpLmxlbmd0aCA9PSAwXG4gICAgICAgICAgfHwgdGhpcy5ncmFwaE1hbmFnZXIuaW5jbHVkZXNJbnZhbGlkRWRnZSgpKVxuICB7XG4gICAgaXNMYXlvdXRTdWNjZXNzZnVsbCA9IGZhbHNlO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIC8vIGNhbGN1bGF0ZSBleGVjdXRpb24gdGltZVxuICAgIHZhciBzdGFydFRpbWUgPSAwO1xuXG4gICAgaWYgKCF0aGlzLmlzU3ViTGF5b3V0KVxuICAgIHtcbiAgICAgIHN0YXJ0VGltZSA9IG5ldyBEYXRlKCkuZ2V0VGltZSgpXG4gICAgfVxuXG4gICAgaXNMYXlvdXRTdWNjZXNzZnVsbCA9IHRoaXMubGF5b3V0KCk7XG5cbiAgICBpZiAoIXRoaXMuaXNTdWJMYXlvdXQpXG4gICAge1xuICAgICAgdmFyIGVuZFRpbWUgPSBuZXcgRGF0ZSgpLmdldFRpbWUoKTtcbiAgICAgIHZhciBleGNUaW1lID0gZW5kVGltZSAtIHN0YXJ0VGltZTtcblxuICAgICAgY29uc29sZS5sb2coXCJUb3RhbCBleGVjdXRpb24gdGltZTogXCIgKyBleGNUaW1lICsgXCIgbWlsaXNlY29uZHMuXCIpO1xuICAgIH1cbiAgfVxuXG4gIGlmIChpc0xheW91dFN1Y2Nlc3NmdWxsKVxuICB7XG4gICAgaWYgKCF0aGlzLmlzU3ViTGF5b3V0KVxuICAgIHtcbiAgICAgIHRoaXMuZG9Qb3N0TGF5b3V0KCk7XG4gICAgfVxuICB9XG5cbiAgdGhpcy5pc0xheW91dEZpbmlzaGVkID0gdHJ1ZTtcblxuICByZXR1cm4gaXNMYXlvdXRTdWNjZXNzZnVsbDtcbn07XG5cbi8qKlxuICogVGhpcyBtZXRob2QgcGVyZm9ybXMgdGhlIG9wZXJhdGlvbnMgcmVxdWlyZWQgYWZ0ZXIgbGF5b3V0LlxuICovXG5MYXlvdXQucHJvdG90eXBlLmRvUG9zdExheW91dCA9IGZ1bmN0aW9uICgpXG57XG4gIC8vYXNzZXJ0ICFpc1N1YkxheW91dCA6IFwiU2hvdWxkIG5vdCBiZSBjYWxsZWQgb24gc3ViLWxheW91dCFcIjtcbiAgLy8gUHJvcGFnYXRlIGdlb21ldHJpYyBjaGFuZ2VzIHRvIHYtbGV2ZWwgb2JqZWN0c1xuICB0aGlzLnRyYW5zZm9ybSgpO1xuICB0aGlzLnVwZGF0ZSgpO1xufTtcblxuLyoqXG4gKiBUaGlzIG1ldGhvZCB1cGRhdGVzIHRoZSBnZW9tZXRyeSBvZiB0aGUgdGFyZ2V0IGdyYXBoIGFjY29yZGluZyB0b1xuICogY2FsY3VsYXRlZCBsYXlvdXQuXG4gKi9cbkxheW91dC5wcm90b3R5cGUudXBkYXRlMiA9IGZ1bmN0aW9uICgpIHtcbiAgLy8gdXBkYXRlIGJlbmQgcG9pbnRzXG4gIGlmICh0aGlzLmNyZWF0ZUJlbmRzQXNOZWVkZWQpXG4gIHtcbiAgICB0aGlzLmNyZWF0ZUJlbmRwb2ludHNGcm9tRHVtbXlOb2RlcygpO1xuXG4gICAgLy8gcmVzZXQgYWxsIGVkZ2VzLCBzaW5jZSB0aGUgdG9wb2xvZ3kgaGFzIGNoYW5nZWRcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5yZXNldEFsbEVkZ2VzKCk7XG4gIH1cblxuICAvLyBwZXJmb3JtIGVkZ2UsIG5vZGUgYW5kIHJvb3QgdXBkYXRlcyBpZiBsYXlvdXQgaXMgbm90IGNhbGxlZFxuICAvLyByZW1vdGVseVxuICBpZiAoIXRoaXMuaXNSZW1vdGVVc2UpXG4gIHtcbiAgICAvLyB1cGRhdGUgYWxsIGVkZ2VzXG4gICAgdmFyIGVkZ2U7XG4gICAgdmFyIGFsbEVkZ2VzID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsRWRnZXMoKTtcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IGFsbEVkZ2VzLmxlbmd0aDsgaSsrKVxuICAgIHtcbiAgICAgIGVkZ2UgPSBhbGxFZGdlc1tpXTtcbi8vICAgICAgdGhpcy51cGRhdGUoZWRnZSk7XG4gICAgfVxuXG4gICAgLy8gcmVjdXJzaXZlbHkgdXBkYXRlIG5vZGVzXG4gICAgdmFyIG5vZGU7XG4gICAgdmFyIG5vZGVzID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldE5vZGVzKCk7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKylcbiAgICB7XG4gICAgICBub2RlID0gbm9kZXNbaV07XG4vLyAgICAgIHRoaXMudXBkYXRlKG5vZGUpO1xuICAgIH1cblxuICAgIC8vIHVwZGF0ZSByb290IGdyYXBoXG4gICAgdGhpcy51cGRhdGUodGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpKTtcbiAgfVxufTtcblxuTGF5b3V0LnByb3RvdHlwZS51cGRhdGUgPSBmdW5jdGlvbiAob2JqKSB7XG4gIGlmIChvYmogPT0gbnVsbCkge1xuICAgIHRoaXMudXBkYXRlMigpO1xuICB9XG4gIGVsc2UgaWYgKG9iaiBpbnN0YW5jZW9mIExOb2RlKSB7XG4gICAgdmFyIG5vZGUgPSBvYmo7XG4gICAgaWYgKG5vZGUuZ2V0Q2hpbGQoKSAhPSBudWxsKVxuICAgIHtcbiAgICAgIC8vIHNpbmNlIG5vZGUgaXMgY29tcG91bmQsIHJlY3Vyc2l2ZWx5IHVwZGF0ZSBjaGlsZCBub2Rlc1xuICAgICAgdmFyIG5vZGVzID0gbm9kZS5nZXRDaGlsZCgpLmdldE5vZGVzKCk7XG4gICAgICBmb3IgKHZhciBpID0gMDsgaSA8IG5vZGVzLmxlbmd0aDsgaSsrKVxuICAgICAge1xuICAgICAgICB1cGRhdGUobm9kZXNbaV0pO1xuICAgICAgfVxuICAgIH1cblxuICAgIC8vIGlmIHRoZSBsLWxldmVsIG5vZGUgaXMgYXNzb2NpYXRlZCB3aXRoIGEgdi1sZXZlbCBncmFwaCBvYmplY3QsXG4gICAgLy8gdGhlbiBpdCBpcyBhc3N1bWVkIHRoYXQgdGhlIHYtbGV2ZWwgbm9kZSBpbXBsZW1lbnRzIHRoZVxuICAgIC8vIGludGVyZmFjZSBVcGRhdGFibGUuXG4gICAgaWYgKG5vZGUudkdyYXBoT2JqZWN0ICE9IG51bGwpXG4gICAge1xuICAgICAgLy8gY2FzdCB0byBVcGRhdGFibGUgd2l0aG91dCBhbnkgdHlwZSBjaGVja1xuICAgICAgdmFyIHZOb2RlID0gbm9kZS52R3JhcGhPYmplY3Q7XG5cbiAgICAgIC8vIGNhbGwgdGhlIHVwZGF0ZSBtZXRob2Qgb2YgdGhlIGludGVyZmFjZVxuICAgICAgdk5vZGUudXBkYXRlKG5vZGUpO1xuICAgIH1cbiAgfVxuICBlbHNlIGlmIChvYmogaW5zdGFuY2VvZiBMRWRnZSkge1xuICAgIHZhciBlZGdlID0gb2JqO1xuICAgIC8vIGlmIHRoZSBsLWxldmVsIGVkZ2UgaXMgYXNzb2NpYXRlZCB3aXRoIGEgdi1sZXZlbCBncmFwaCBvYmplY3QsXG4gICAgLy8gdGhlbiBpdCBpcyBhc3N1bWVkIHRoYXQgdGhlIHYtbGV2ZWwgZWRnZSBpbXBsZW1lbnRzIHRoZVxuICAgIC8vIGludGVyZmFjZSBVcGRhdGFibGUuXG5cbiAgICBpZiAoZWRnZS52R3JhcGhPYmplY3QgIT0gbnVsbClcbiAgICB7XG4gICAgICAvLyBjYXN0IHRvIFVwZGF0YWJsZSB3aXRob3V0IGFueSB0eXBlIGNoZWNrXG4gICAgICB2YXIgdkVkZ2UgPSBlZGdlLnZHcmFwaE9iamVjdDtcblxuICAgICAgLy8gY2FsbCB0aGUgdXBkYXRlIG1ldGhvZCBvZiB0aGUgaW50ZXJmYWNlXG4gICAgICB2RWRnZS51cGRhdGUoZWRnZSk7XG4gICAgfVxuICB9XG4gIGVsc2UgaWYgKG9iaiBpbnN0YW5jZW9mIExHcmFwaCkge1xuICAgIHZhciBncmFwaCA9IG9iajtcbiAgICAvLyBpZiB0aGUgbC1sZXZlbCBncmFwaCBpcyBhc3NvY2lhdGVkIHdpdGggYSB2LWxldmVsIGdyYXBoIG9iamVjdCxcbiAgICAvLyB0aGVuIGl0IGlzIGFzc3VtZWQgdGhhdCB0aGUgdi1sZXZlbCBvYmplY3QgaW1wbGVtZW50cyB0aGVcbiAgICAvLyBpbnRlcmZhY2UgVXBkYXRhYmxlLlxuXG4gICAgaWYgKGdyYXBoLnZHcmFwaE9iamVjdCAhPSBudWxsKVxuICAgIHtcbiAgICAgIC8vIGNhc3QgdG8gVXBkYXRhYmxlIHdpdGhvdXQgYW55IHR5cGUgY2hlY2tcbiAgICAgIHZhciB2R3JhcGggPSBncmFwaC52R3JhcGhPYmplY3Q7XG5cbiAgICAgIC8vIGNhbGwgdGhlIHVwZGF0ZSBtZXRob2Qgb2YgdGhlIGludGVyZmFjZVxuICAgICAgdkdyYXBoLnVwZGF0ZShncmFwaCk7XG4gICAgfVxuICB9XG59O1xuXG4vKipcbiAqIFRoaXMgbWV0aG9kIGlzIHVzZWQgdG8gc2V0IGFsbCBsYXlvdXQgcGFyYW1ldGVycyB0byBkZWZhdWx0IHZhbHVlc1xuICogZGV0ZXJtaW5lZCBhdCBjb21waWxlIHRpbWUuXG4gKi9cbkxheW91dC5wcm90b3R5cGUuaW5pdFBhcmFtZXRlcnMgPSBmdW5jdGlvbiAoKSB7XG4gIGlmICghdGhpcy5pc1N1YkxheW91dClcbiAge1xuICAgIHRoaXMubGF5b3V0UXVhbGl0eSA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX1FVQUxJVFk7XG4gICAgdGhpcy5hbmltYXRpb25EdXJpbmdMYXlvdXQgPSBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fT05fTEFZT1VUO1xuICAgIHRoaXMuYW5pbWF0aW9uUGVyaW9kID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX1BFUklPRDtcbiAgICB0aGlzLmFuaW1hdGlvbk9uTGF5b3V0ID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX0RVUklOR19MQVlPVVQ7XG4gICAgdGhpcy5pbmNyZW1lbnRhbCA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMO1xuICAgIHRoaXMuY3JlYXRlQmVuZHNBc05lZWRlZCA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX0NSRUFURV9CRU5EU19BU19ORUVERUQ7XG4gICAgdGhpcy51bmlmb3JtTGVhZk5vZGVTaXplcyA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX1VOSUZPUk1fTEVBRl9OT0RFX1NJWkVTO1xuICB9XG5cbiAgaWYgKHRoaXMuYW5pbWF0aW9uRHVyaW5nTGF5b3V0KVxuICB7XG4gICAgYW5pbWF0aW9uT25MYXlvdXQgPSBmYWxzZTtcbiAgfVxufTtcblxuTGF5b3V0LnByb3RvdHlwZS50cmFuc2Zvcm0gPSBmdW5jdGlvbiAobmV3TGVmdFRvcCkge1xuICBpZiAobmV3TGVmdFRvcCA9PSB1bmRlZmluZWQpIHtcbiAgICB0aGlzLnRyYW5zZm9ybShuZXcgUG9pbnREKDAsIDApKTtcbiAgfVxuICBlbHNlIHtcbiAgICAvLyBjcmVhdGUgYSB0cmFuc2Zvcm1hdGlvbiBvYmplY3QgKGZyb20gRWNsaXBzZSB0byBsYXlvdXQpLiBXaGVuIGFuXG4gICAgLy8gaW52ZXJzZSB0cmFuc2Zvcm0gaXMgYXBwbGllZCwgd2UgZ2V0IHVwcGVyLWxlZnQgY29vcmRpbmF0ZSBvZiB0aGVcbiAgICAvLyBkcmF3aW5nIG9yIHRoZSByb290IGdyYXBoIGF0IGdpdmVuIGlucHV0IGNvb3JkaW5hdGUgKHNvbWUgbWFyZ2luc1xuICAgIC8vIGFscmVhZHkgaW5jbHVkZWQgaW4gY2FsY3VsYXRpb24gb2YgbGVmdC10b3ApLlxuXG4gICAgdmFyIHRyYW5zID0gbmV3IFRyYW5zZm9ybSgpO1xuICAgIHZhciBsZWZ0VG9wID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLnVwZGF0ZUxlZnRUb3AoKTtcblxuICAgIGlmIChsZWZ0VG9wICE9IG51bGwpXG4gICAge1xuICAgICAgdHJhbnMuc2V0V29ybGRPcmdYKG5ld0xlZnRUb3AueCk7XG4gICAgICB0cmFucy5zZXRXb3JsZE9yZ1kobmV3TGVmdFRvcC55KTtcblxuICAgICAgdHJhbnMuc2V0RGV2aWNlT3JnWChsZWZ0VG9wLngpO1xuICAgICAgdHJhbnMuc2V0RGV2aWNlT3JnWShsZWZ0VG9wLnkpO1xuXG4gICAgICB2YXIgbm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCk7XG4gICAgICB2YXIgbm9kZTtcblxuICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKylcbiAgICAgIHtcbiAgICAgICAgbm9kZSA9IG5vZGVzW2ldO1xuICAgICAgICBub2RlLnRyYW5zZm9ybSh0cmFucyk7XG4gICAgICB9XG4gICAgfVxuICB9XG59O1xuXG5MYXlvdXQucHJvdG90eXBlLnBvc2l0aW9uTm9kZXNSYW5kb21seSA9IGZ1bmN0aW9uIChncmFwaCkge1xuXG4gIGlmIChncmFwaCA9PSB1bmRlZmluZWQpIHtcbiAgICAvL2Fzc2VydCAhdGhpcy5pbmNyZW1lbnRhbDtcbiAgICB0aGlzLnBvc2l0aW9uTm9kZXNSYW5kb21seSh0aGlzLmdldEdyYXBoTWFuYWdlcigpLmdldFJvb3QoKSk7XG4gICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRSb290KCkudXBkYXRlQm91bmRzKHRydWUpO1xuICB9XG4gIGVsc2Uge1xuICAgIHZhciBsTm9kZTtcbiAgICB2YXIgY2hpbGRHcmFwaDtcblxuICAgIHZhciBub2RlcyA9IGdyYXBoLmdldE5vZGVzKCk7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKylcbiAgICB7XG4gICAgICBsTm9kZSA9IG5vZGVzW2ldO1xuICAgICAgY2hpbGRHcmFwaCA9IGxOb2RlLmdldENoaWxkKCk7XG5cbiAgICAgIGlmIChjaGlsZEdyYXBoID09IG51bGwpXG4gICAgICB7XG4gICAgICAgIGxOb2RlLnNjYXR0ZXIoKTtcbiAgICAgIH1cbiAgICAgIGVsc2UgaWYgKGNoaWxkR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGggPT0gMClcbiAgICAgIHtcbiAgICAgICAgbE5vZGUuc2NhdHRlcigpO1xuICAgICAgfVxuICAgICAgZWxzZVxuICAgICAge1xuICAgICAgICB0aGlzLnBvc2l0aW9uTm9kZXNSYW5kb21seShjaGlsZEdyYXBoKTtcbiAgICAgICAgbE5vZGUudXBkYXRlQm91bmRzKCk7XG4gICAgICB9XG4gICAgfVxuICB9XG59O1xuXG4vKipcbiAqIFRoaXMgbWV0aG9kIHJldHVybnMgYSBsaXN0IG9mIHRyZWVzIHdoZXJlIGVhY2ggdHJlZSBpcyByZXByZXNlbnRlZCBhcyBhXG4gKiBsaXN0IG9mIGwtbm9kZXMuIFRoZSBtZXRob2QgcmV0dXJucyBhIGxpc3Qgb2Ygc2l6ZSAwIHdoZW46XG4gKiAtIFRoZSBncmFwaCBpcyBub3QgZmxhdCBvclxuICogLSBPbmUgb2YgdGhlIGNvbXBvbmVudChzKSBvZiB0aGUgZ3JhcGggaXMgbm90IGEgdHJlZS5cbiAqL1xuTGF5b3V0LnByb3RvdHlwZS5nZXRGbGF0Rm9yZXN0ID0gZnVuY3Rpb24gKClcbntcbiAgdmFyIGZsYXRGb3Jlc3QgPSBbXTtcbiAgdmFyIGlzRm9yZXN0ID0gdHJ1ZTtcblxuICAvLyBRdWljayByZWZlcmVuY2UgZm9yIGFsbCBub2RlcyBpbiB0aGUgZ3JhcGggbWFuYWdlciBhc3NvY2lhdGVkIHdpdGhcbiAgLy8gdGhpcyBsYXlvdXQuIFRoZSBsaXN0IHNob3VsZCBub3QgYmUgY2hhbmdlZC5cbiAgdmFyIGFsbE5vZGVzID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldE5vZGVzKCk7XG5cbiAgLy8gRmlyc3QgYmUgc3VyZSB0aGF0IHRoZSBncmFwaCBpcyBmbGF0XG4gIHZhciBpc0ZsYXQgPSB0cnVlO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgYWxsTm9kZXMubGVuZ3RoOyBpKyspXG4gIHtcbiAgICBpZiAoYWxsTm9kZXNbaV0uZ2V0Q2hpbGQoKSAhPSBudWxsKVxuICAgIHtcbiAgICAgIGlzRmxhdCA9IGZhbHNlO1xuICAgIH1cbiAgfVxuXG4gIC8vIFJldHVybiBlbXB0eSBmb3Jlc3QgaWYgdGhlIGdyYXBoIGlzIG5vdCBmbGF0LlxuICBpZiAoIWlzRmxhdClcbiAge1xuICAgIHJldHVybiBmbGF0Rm9yZXN0O1xuICB9XG5cbiAgLy8gUnVuIEJGUyBmb3IgZWFjaCBjb21wb25lbnQgb2YgdGhlIGdyYXBoLlxuXG4gIHZhciB2aXNpdGVkID0gbmV3IEhhc2hTZXQoKTtcbiAgdmFyIHRvQmVWaXNpdGVkID0gW107XG4gIHZhciBwYXJlbnRzID0gbmV3IEhhc2hNYXAoKTtcbiAgdmFyIHVuUHJvY2Vzc2VkTm9kZXMgPSBbXTtcblxuICB1blByb2Nlc3NlZE5vZGVzID0gdW5Qcm9jZXNzZWROb2Rlcy5jb25jYXQoYWxsTm9kZXMpO1xuXG4gIC8vIEVhY2ggaXRlcmF0aW9uIG9mIHRoaXMgbG9vcCBmaW5kcyBhIGNvbXBvbmVudCBvZiB0aGUgZ3JhcGggYW5kXG4gIC8vIGRlY2lkZXMgd2hldGhlciBpdCBpcyBhIHRyZWUgb3Igbm90LiBJZiBpdCBpcyBhIHRyZWUsIGFkZHMgaXQgdG8gdGhlXG4gIC8vIGZvcmVzdCBhbmQgY29udGludWVkIHdpdGggdGhlIG5leHQgY29tcG9uZW50LlxuXG4gIHdoaWxlICh1blByb2Nlc3NlZE5vZGVzLmxlbmd0aCA+IDAgJiYgaXNGb3Jlc3QpXG4gIHtcbiAgICB0b0JlVmlzaXRlZC5wdXNoKHVuUHJvY2Vzc2VkTm9kZXNbMF0pO1xuXG4gICAgLy8gU3RhcnQgdGhlIEJGUy4gRWFjaCBpdGVyYXRpb24gb2YgdGhpcyBsb29wIHZpc2l0cyBhIG5vZGUgaW4gYVxuICAgIC8vIEJGUyBtYW5uZXIuXG4gICAgd2hpbGUgKHRvQmVWaXNpdGVkLmxlbmd0aCA+IDAgJiYgaXNGb3Jlc3QpXG4gICAge1xuICAgICAgLy9wb29sIG9wZXJhdGlvblxuICAgICAgdmFyIGN1cnJlbnROb2RlID0gdG9CZVZpc2l0ZWRbMF07XG4gICAgICB0b0JlVmlzaXRlZC5zcGxpY2UoMCwgMSk7XG4gICAgICB2aXNpdGVkLmFkZChjdXJyZW50Tm9kZSk7XG5cbiAgICAgIC8vIFRyYXZlcnNlIGFsbCBuZWlnaGJvcnMgb2YgdGhpcyBub2RlXG4gICAgICB2YXIgbmVpZ2hib3JFZGdlcyA9IGN1cnJlbnROb2RlLmdldEVkZ2VzKCk7XG5cbiAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbmVpZ2hib3JFZGdlcy5sZW5ndGg7IGkrKylcbiAgICAgIHtcbiAgICAgICAgdmFyIGN1cnJlbnROZWlnaGJvciA9XG4gICAgICAgICAgICAgICAgbmVpZ2hib3JFZGdlc1tpXS5nZXRPdGhlckVuZChjdXJyZW50Tm9kZSk7XG5cbiAgICAgICAgLy8gSWYgQkZTIGlzIG5vdCBncm93aW5nIGZyb20gdGhpcyBuZWlnaGJvci5cbiAgICAgICAgaWYgKHBhcmVudHMuZ2V0KGN1cnJlbnROb2RlKSAhPSBjdXJyZW50TmVpZ2hib3IpXG4gICAgICAgIHtcbiAgICAgICAgICAvLyBXZSBoYXZlbid0IHByZXZpb3VzbHkgdmlzaXRlZCB0aGlzIG5laWdoYm9yLlxuICAgICAgICAgIGlmICghdmlzaXRlZC5jb250YWlucyhjdXJyZW50TmVpZ2hib3IpKVxuICAgICAgICAgIHtcbiAgICAgICAgICAgIHRvQmVWaXNpdGVkLnB1c2goY3VycmVudE5laWdoYm9yKTtcbiAgICAgICAgICAgIHBhcmVudHMucHV0KGN1cnJlbnROZWlnaGJvciwgY3VycmVudE5vZGUpO1xuICAgICAgICAgIH1cbiAgICAgICAgICAvLyBTaW5jZSB3ZSBoYXZlIHByZXZpb3VzbHkgdmlzaXRlZCB0aGlzIG5laWdoYm9yIGFuZFxuICAgICAgICAgIC8vIHRoaXMgbmVpZ2hib3IgaXMgbm90IHBhcmVudCBvZiBjdXJyZW50Tm9kZSwgZ2l2ZW5cbiAgICAgICAgICAvLyBncmFwaCBjb250YWlucyBhIGNvbXBvbmVudCB0aGF0IGlzIG5vdCB0cmVlLCBoZW5jZVxuICAgICAgICAgIC8vIGl0IGlzIG5vdCBhIGZvcmVzdC5cbiAgICAgICAgICBlbHNlXG4gICAgICAgICAge1xuICAgICAgICAgICAgaXNGb3Jlc3QgPSBmYWxzZTtcbiAgICAgICAgICAgIGJyZWFrO1xuICAgICAgICAgIH1cbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cblxuICAgIC8vIFRoZSBncmFwaCBjb250YWlucyBhIGNvbXBvbmVudCB0aGF0IGlzIG5vdCBhIHRyZWUuIEVtcHR5XG4gICAgLy8gcHJldmlvdXNseSBmb3VuZCB0cmVlcy4gVGhlIG1ldGhvZCB3aWxsIGVuZC5cbiAgICBpZiAoIWlzRm9yZXN0KVxuICAgIHtcbiAgICAgIGZsYXRGb3Jlc3QgPSBbXTtcbiAgICB9XG4gICAgLy8gU2F2ZSBjdXJyZW50bHkgdmlzaXRlZCBub2RlcyBhcyBhIHRyZWUgaW4gb3VyIGZvcmVzdC4gUmVzZXRcbiAgICAvLyB2aXNpdGVkIGFuZCBwYXJlbnRzIGxpc3RzLiBDb250aW51ZSB3aXRoIHRoZSBuZXh0IGNvbXBvbmVudCBvZlxuICAgIC8vIHRoZSBncmFwaCwgaWYgYW55LlxuICAgIGVsc2VcbiAgICB7XG4gICAgICB2YXIgdGVtcCA9IFtdO1xuICAgICAgdmlzaXRlZC5hZGRBbGxUbyh0ZW1wKTtcbiAgICAgIGZsYXRGb3Jlc3QucHVzaCh0ZW1wKTtcbiAgICAgIC8vZmxhdEZvcmVzdCA9IGZsYXRGb3Jlc3QuY29uY2F0KHRlbXApO1xuICAgICAgLy91blByb2Nlc3NlZE5vZGVzLnJlbW92ZUFsbCh2aXNpdGVkKTtcbiAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGVtcC5sZW5ndGg7IGkrKykge1xuICAgICAgICB2YXIgdmFsdWUgPSB0ZW1wW2ldO1xuICAgICAgICB2YXIgaW5kZXggPSB1blByb2Nlc3NlZE5vZGVzLmluZGV4T2YodmFsdWUpO1xuICAgICAgICBpZiAoaW5kZXggPiAtMSkge1xuICAgICAgICAgIHVuUHJvY2Vzc2VkTm9kZXMuc3BsaWNlKGluZGV4LCAxKTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgICAgdmlzaXRlZCA9IG5ldyBIYXNoU2V0KCk7XG4gICAgICBwYXJlbnRzID0gbmV3IEhhc2hNYXAoKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gZmxhdEZvcmVzdDtcbn07XG5cbi8qKlxuICogVGhpcyBtZXRob2QgY3JlYXRlcyBkdW1teSBub2RlcyAoYW4gbC1sZXZlbCBub2RlIHdpdGggbWluaW1hbCBkaW1lbnNpb25zKVxuICogZm9yIHRoZSBnaXZlbiBlZGdlIChvbmUgcGVyIGJlbmRwb2ludCkuIFRoZSBleGlzdGluZyBsLWxldmVsIHN0cnVjdHVyZVxuICogaXMgdXBkYXRlZCBhY2NvcmRpbmdseS5cbiAqL1xuTGF5b3V0LnByb3RvdHlwZS5jcmVhdGVEdW1teU5vZGVzRm9yQmVuZHBvaW50cyA9IGZ1bmN0aW9uIChlZGdlKVxue1xuICB2YXIgZHVtbXlOb2RlcyA9IFtdO1xuICB2YXIgcHJldiA9IGVkZ2Uuc291cmNlO1xuXG4gIHZhciBncmFwaCA9IHRoaXMuZ3JhcGhNYW5hZ2VyLmNhbGNMb3dlc3RDb21tb25BbmNlc3RvcihlZGdlLnNvdXJjZSwgZWRnZS50YXJnZXQpO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgZWRnZS5iZW5kcG9pbnRzLmxlbmd0aDsgaSsrKVxuICB7XG4gICAgLy8gY3JlYXRlIG5ldyBkdW1teSBub2RlXG4gICAgdmFyIGR1bW15Tm9kZSA9IHRoaXMubmV3Tm9kZShudWxsKTtcbiAgICBkdW1teU5vZGUuc2V0UmVjdChuZXcgUG9pbnQoMCwgMCksIG5ldyBEaW1lbnNpb24oMSwgMSkpO1xuXG4gICAgZ3JhcGguYWRkKGR1bW15Tm9kZSk7XG5cbiAgICAvLyBjcmVhdGUgbmV3IGR1bW15IGVkZ2UgYmV0d2VlbiBwcmV2IGFuZCBkdW1teSBub2RlXG4gICAgdmFyIGR1bW15RWRnZSA9IHRoaXMubmV3RWRnZShudWxsKTtcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5hZGQoZHVtbXlFZGdlLCBwcmV2LCBkdW1teU5vZGUpO1xuXG4gICAgZHVtbXlOb2Rlcy5hZGQoZHVtbXlOb2RlKTtcbiAgICBwcmV2ID0gZHVtbXlOb2RlO1xuICB9XG5cbiAgdmFyIGR1bW15RWRnZSA9IHRoaXMubmV3RWRnZShudWxsKTtcbiAgdGhpcy5ncmFwaE1hbmFnZXIuYWRkKGR1bW15RWRnZSwgcHJldiwgZWRnZS50YXJnZXQpO1xuXG4gIHRoaXMuZWRnZVRvRHVtbXlOb2Rlcy5wdXQoZWRnZSwgZHVtbXlOb2Rlcyk7XG5cbiAgLy8gcmVtb3ZlIHJlYWwgZWRnZSBmcm9tIGdyYXBoIG1hbmFnZXIgaWYgaXQgaXMgaW50ZXItZ3JhcGhcbiAgaWYgKGVkZ2UuaXNJbnRlckdyYXBoKCkpXG4gIHtcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5yZW1vdmUoZWRnZSk7XG4gIH1cbiAgLy8gZWxzZSwgcmVtb3ZlIHRoZSBlZGdlIGZyb20gdGhlIGN1cnJlbnQgZ3JhcGhcbiAgZWxzZVxuICB7XG4gICAgZ3JhcGgucmVtb3ZlKGVkZ2UpO1xuICB9XG5cbiAgcmV0dXJuIGR1bW15Tm9kZXM7XG59O1xuXG4vKipcbiAqIFRoaXMgbWV0aG9kIGNyZWF0ZXMgYmVuZHBvaW50cyBmb3IgZWRnZXMgZnJvbSB0aGUgZHVtbXkgbm9kZXNcbiAqIGF0IGwtbGV2ZWwuXG4gKi9cbkxheW91dC5wcm90b3R5cGUuY3JlYXRlQmVuZHBvaW50c0Zyb21EdW1teU5vZGVzID0gZnVuY3Rpb24gKClcbntcbiAgdmFyIGVkZ2VzID0gW107XG4gIGVkZ2VzID0gZWRnZXMuY29uY2F0KHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEFsbEVkZ2VzKCkpO1xuICBlZGdlcyA9IHRoaXMuZWRnZVRvRHVtbXlOb2Rlcy5rZXlTZXQoKS5jb25jYXQoZWRnZXMpO1xuXG4gIGZvciAodmFyIGsgPSAwOyBrIDwgZWRnZXMubGVuZ3RoOyBrKyspXG4gIHtcbiAgICB2YXIgbEVkZ2UgPSBlZGdlc1trXTtcblxuICAgIGlmIChsRWRnZS5iZW5kcG9pbnRzLmxlbmd0aCA+IDApXG4gICAge1xuICAgICAgdmFyIHBhdGggPSB0aGlzLmVkZ2VUb0R1bW15Tm9kZXMuZ2V0KGxFZGdlKTtcblxuICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBwYXRoLmxlbmd0aDsgaSsrKVxuICAgICAge1xuICAgICAgICB2YXIgZHVtbXlOb2RlID0gcGF0aFtpXTtcbiAgICAgICAgdmFyIHAgPSBuZXcgUG9pbnREKGR1bW15Tm9kZS5nZXRDZW50ZXJYKCksXG4gICAgICAgICAgICAgICAgZHVtbXlOb2RlLmdldENlbnRlclkoKSk7XG5cbiAgICAgICAgLy8gdXBkYXRlIGJlbmRwb2ludCdzIGxvY2F0aW9uIGFjY29yZGluZyB0byBkdW1teSBub2RlXG4gICAgICAgIHZhciBlYnAgPSBsRWRnZS5iZW5kcG9pbnRzLmdldChpKTtcbiAgICAgICAgZWJwLnggPSBwLng7XG4gICAgICAgIGVicC55ID0gcC55O1xuXG4gICAgICAgIC8vIHJlbW92ZSB0aGUgZHVtbXkgbm9kZSwgZHVtbXkgZWRnZXMgaW5jaWRlbnQgd2l0aCB0aGlzXG4gICAgICAgIC8vIGR1bW15IG5vZGUgaXMgYWxzbyByZW1vdmVkICh3aXRoaW4gdGhlIHJlbW92ZSBtZXRob2QpXG4gICAgICAgIGR1bW15Tm9kZS5nZXRPd25lcigpLnJlbW92ZShkdW1teU5vZGUpO1xuICAgICAgfVxuXG4gICAgICAvLyBhZGQgdGhlIHJlYWwgZWRnZSB0byBncmFwaFxuICAgICAgdGhpcy5ncmFwaE1hbmFnZXIuYWRkKGxFZGdlLCBsRWRnZS5zb3VyY2UsIGxFZGdlLnRhcmdldCk7XG4gICAgfVxuICB9XG59O1xuXG5MYXlvdXQudHJhbnNmb3JtID0gZnVuY3Rpb24gKHNsaWRlclZhbHVlLCBkZWZhdWx0VmFsdWUsIG1pbkRpdiwgbWF4TXVsKSB7XG4gIGlmIChtaW5EaXYgIT0gdW5kZWZpbmVkICYmIG1heE11bCAhPSB1bmRlZmluZWQpIHtcbiAgICB2YXIgdmFsdWUgPSBkZWZhdWx0VmFsdWU7XG5cbiAgICBpZiAoc2xpZGVyVmFsdWUgPD0gNTApXG4gICAge1xuICAgICAgdmFyIG1pblZhbHVlID0gZGVmYXVsdFZhbHVlIC8gbWluRGl2O1xuICAgICAgdmFsdWUgLT0gKChkZWZhdWx0VmFsdWUgLSBtaW5WYWx1ZSkgLyA1MCkgKiAoNTAgLSBzbGlkZXJWYWx1ZSk7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICB2YXIgbWF4VmFsdWUgPSBkZWZhdWx0VmFsdWUgKiBtYXhNdWw7XG4gICAgICB2YWx1ZSArPSAoKG1heFZhbHVlIC0gZGVmYXVsdFZhbHVlKSAvIDUwKSAqIChzbGlkZXJWYWx1ZSAtIDUwKTtcbiAgICB9XG5cbiAgICByZXR1cm4gdmFsdWU7XG4gIH1cbiAgZWxzZSB7XG4gICAgdmFyIGEsIGI7XG5cbiAgICBpZiAoc2xpZGVyVmFsdWUgPD0gNTApXG4gICAge1xuICAgICAgYSA9IDkuMCAqIGRlZmF1bHRWYWx1ZSAvIDUwMC4wO1xuICAgICAgYiA9IGRlZmF1bHRWYWx1ZSAvIDEwLjA7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICBhID0gOS4wICogZGVmYXVsdFZhbHVlIC8gNTAuMDtcbiAgICAgIGIgPSAtOCAqIGRlZmF1bHRWYWx1ZTtcbiAgICB9XG5cbiAgICByZXR1cm4gKGEgKiBzbGlkZXJWYWx1ZSArIGIpO1xuICB9XG59O1xuXG4vKipcbiAqIFRoaXMgbWV0aG9kIGZpbmRzIGFuZCByZXR1cm5zIHRoZSBjZW50ZXIgb2YgdGhlIGdpdmVuIG5vZGVzLCBhc3N1bWluZ1xuICogdGhhdCB0aGUgZ2l2ZW4gbm9kZXMgZm9ybSBhIHRyZWUgaW4gdGhlbXNlbHZlcy5cbiAqL1xuTGF5b3V0LmZpbmRDZW50ZXJPZlRyZWUgPSBmdW5jdGlvbiAobm9kZXMpXG57XG4gIHZhciBsaXN0ID0gW107XG4gIGxpc3QgPSBsaXN0LmNvbmNhdChub2Rlcyk7XG5cbiAgdmFyIHJlbW92ZWROb2RlcyA9IFtdO1xuICB2YXIgcmVtYWluaW5nRGVncmVlcyA9IG5ldyBIYXNoTWFwKCk7XG4gIHZhciBmb3VuZENlbnRlciA9IGZhbHNlO1xuICB2YXIgY2VudGVyTm9kZSA9IG51bGw7XG5cbiAgaWYgKGxpc3QubGVuZ3RoID09IDEgfHwgbGlzdC5sZW5ndGggPT0gMilcbiAge1xuICAgIGZvdW5kQ2VudGVyID0gdHJ1ZTtcbiAgICBjZW50ZXJOb2RlID0gbGlzdFswXTtcbiAgfVxuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgbGlzdC5sZW5ndGg7IGkrKylcbiAge1xuICAgIHZhciBub2RlID0gbGlzdFtpXTtcbiAgICB2YXIgZGVncmVlID0gbm9kZS5nZXROZWlnaGJvcnNMaXN0KCkuc2l6ZSgpO1xuICAgIHJlbWFpbmluZ0RlZ3JlZXMucHV0KG5vZGUsIG5vZGUuZ2V0TmVpZ2hib3JzTGlzdCgpLnNpemUoKSk7XG5cbiAgICBpZiAoZGVncmVlID09IDEpXG4gICAge1xuICAgICAgcmVtb3ZlZE5vZGVzLnB1c2gobm9kZSk7XG4gICAgfVxuICB9XG5cbiAgdmFyIHRlbXBMaXN0ID0gW107XG4gIHRlbXBMaXN0ID0gdGVtcExpc3QuY29uY2F0KHJlbW92ZWROb2Rlcyk7XG5cbiAgd2hpbGUgKCFmb3VuZENlbnRlcilcbiAge1xuICAgIHZhciB0ZW1wTGlzdDIgPSBbXTtcbiAgICB0ZW1wTGlzdDIgPSB0ZW1wTGlzdDIuY29uY2F0KHRlbXBMaXN0KTtcbiAgICB0ZW1wTGlzdCA9IFtdO1xuXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsaXN0Lmxlbmd0aDsgaSsrKVxuICAgIHtcbiAgICAgIHZhciBub2RlID0gbGlzdFtpXTtcblxuICAgICAgdmFyIGluZGV4ID0gbGlzdC5pbmRleE9mKG5vZGUpO1xuICAgICAgaWYgKGluZGV4ID49IDApIHtcbiAgICAgICAgbGlzdC5zcGxpY2UoaW5kZXgsIDEpO1xuICAgICAgfVxuXG4gICAgICB2YXIgbmVpZ2hib3VycyA9IG5vZGUuZ2V0TmVpZ2hib3JzTGlzdCgpO1xuXG4gICAgICBmb3IgKHZhciBqIGluIG5laWdoYm91cnMuc2V0KVxuICAgICAge1xuICAgICAgICB2YXIgbmVpZ2hib3VyID0gbmVpZ2hib3Vycy5zZXRbal07XG4gICAgICAgIGlmIChyZW1vdmVkTm9kZXMuaW5kZXhPZihuZWlnaGJvdXIpIDwgMClcbiAgICAgICAge1xuICAgICAgICAgIHZhciBvdGhlckRlZ3JlZSA9IHJlbWFpbmluZ0RlZ3JlZXMuZ2V0KG5laWdoYm91cik7XG4gICAgICAgICAgdmFyIG5ld0RlZ3JlZSA9IG90aGVyRGVncmVlIC0gMTtcblxuICAgICAgICAgIGlmIChuZXdEZWdyZWUgPT0gMSlcbiAgICAgICAgICB7XG4gICAgICAgICAgICB0ZW1wTGlzdC5wdXNoKG5laWdoYm91cik7XG4gICAgICAgICAgfVxuXG4gICAgICAgICAgcmVtYWluaW5nRGVncmVlcy5wdXQobmVpZ2hib3VyLCBuZXdEZWdyZWUpO1xuICAgICAgICB9XG4gICAgICB9XG4gICAgfVxuXG4gICAgcmVtb3ZlZE5vZGVzID0gcmVtb3ZlZE5vZGVzLmNvbmNhdCh0ZW1wTGlzdCk7XG5cbiAgICBpZiAobGlzdC5sZW5ndGggPT0gMSB8fCBsaXN0Lmxlbmd0aCA9PSAyKVxuICAgIHtcbiAgICAgIGZvdW5kQ2VudGVyID0gdHJ1ZTtcbiAgICAgIGNlbnRlck5vZGUgPSBsaXN0WzBdO1xuICAgIH1cbiAgfVxuXG4gIHJldHVybiBjZW50ZXJOb2RlO1xufTtcblxuLyoqXG4gKiBEdXJpbmcgdGhlIGNvYXJzZW5pbmcgcHJvY2VzcywgdGhpcyBsYXlvdXQgbWF5IGJlIHJlZmVyZW5jZWQgYnkgdHdvIGdyYXBoIG1hbmFnZXJzXG4gKiB0aGlzIHNldHRlciBmdW5jdGlvbiBncmFudHMgYWNjZXNzIHRvIGNoYW5nZSB0aGUgY3VycmVudGx5IGJlaW5nIHVzZWQgZ3JhcGggbWFuYWdlclxuICovXG5MYXlvdXQucHJvdG90eXBlLnNldEdyYXBoTWFuYWdlciA9IGZ1bmN0aW9uIChnbSlcbntcbiAgdGhpcy5ncmFwaE1hbmFnZXIgPSBnbTtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gTGF5b3V0O1xuIiwiZnVuY3Rpb24gTGF5b3V0Q29uc3RhbnRzKCkge1xufVxuXG4vKipcbiAqIExheW91dCBRdWFsaXR5XG4gKi9cbkxheW91dENvbnN0YW50cy5QUk9PRl9RVUFMSVRZID0gMDtcbkxheW91dENvbnN0YW50cy5ERUZBVUxUX1FVQUxJVFkgPSAxO1xuTGF5b3V0Q29uc3RhbnRzLkRSQUZUX1FVQUxJVFkgPSAyO1xuXG4vKipcbiAqIERlZmF1bHQgcGFyYW1ldGVyc1xuICovXG5MYXlvdXRDb25zdGFudHMuREVGQVVMVF9DUkVBVEVfQkVORFNfQVNfTkVFREVEID0gZmFsc2U7XG4vL0xheW91dENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID0gdHJ1ZTtcbkxheW91dENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID0gZmFsc2U7XG5MYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fT05fTEFZT1VUID0gdHJ1ZTtcbkxheW91dENvbnN0YW50cy5ERUZBVUxUX0FOSU1BVElPTl9EVVJJTkdfTEFZT1VUID0gZmFsc2U7XG5MYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fUEVSSU9EID0gNTA7XG5MYXlvdXRDb25zdGFudHMuREVGQVVMVF9VTklGT1JNX0xFQUZfTk9ERV9TSVpFUyA9IGZhbHNlO1xuXG4vLyAtLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuLy8gU2VjdGlvbjogR2VuZXJhbCBvdGhlciBjb25zdGFudHNcbi8vIC0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG4vKlxuICogTWFyZ2lucyBvZiBhIGdyYXBoIHRvIGJlIGFwcGxpZWQgb24gYm91ZGluZyByZWN0YW5nbGUgb2YgaXRzIGNvbnRlbnRzLiBXZVxuICogYXNzdW1lIG1hcmdpbnMgb24gYWxsIGZvdXIgc2lkZXMgdG8gYmUgdW5pZm9ybS5cbiAqL1xuTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBUEhfTUFSR0lOID0gMTA7XG5cbi8qXG4gKiBUaGUgaGVpZ2h0IG9mIHRoZSBsYWJlbCBvZiBhIGNvbXBvdW5kLiBXZSBhc3N1bWUgdGhlIGxhYmVsIG9mIGEgY29tcG91bmRcbiAqIG5vZGUgaXMgcGxhY2VkIGF0IHRoZSBib3R0b20gd2l0aCBhIGR5bmFtaWMgd2lkdGggc2FtZSBhcyB0aGUgY29tcG91bmRcbiAqIGl0c2VsZi5cbiAqL1xuTGF5b3V0Q29uc3RhbnRzLkxBQkVMX0hFSUdIVCA9IDIwO1xuXG4vKlxuICogQWRkaXRpb25hbCBtYXJnaW5zIHRoYXQgd2UgbWFpbnRhaW4gYXMgc2FmZXR5IGJ1ZmZlciBmb3Igbm9kZS1ub2RlXG4gKiBvdmVybGFwcy4gQ29tcG91bmQgbm9kZSBsYWJlbHMgYXMgd2VsbCBhcyBncmFwaCBtYXJnaW5zIGFyZSBoYW5kbGVkXG4gKiBzZXBhcmF0ZWx5IVxuICovXG5MYXlvdXRDb25zdGFudHMuQ09NUE9VTkRfTk9ERV9NQVJHSU4gPSA1O1xuXG4vKlxuICogRGVmYXVsdCBkaW1lbnNpb24gb2YgYSBub24tY29tcG91bmQgbm9kZS5cbiAqL1xuTGF5b3V0Q29uc3RhbnRzLlNJTVBMRV9OT0RFX1NJWkUgPSA0MDtcblxuLypcbiAqIERlZmF1bHQgZGltZW5zaW9uIG9mIGEgbm9uLWNvbXBvdW5kIG5vZGUuXG4gKi9cbkxheW91dENvbnN0YW50cy5TSU1QTEVfTk9ERV9IQUxGX1NJWkUgPSBMYXlvdXRDb25zdGFudHMuU0lNUExFX05PREVfU0laRSAvIDI7XG5cbi8qXG4gKiBFbXB0eSBjb21wb3VuZCBub2RlIHNpemUuIFdoZW4gYSBjb21wb3VuZCBub2RlIGlzIGVtcHR5LCBpdHMgYm90aFxuICogZGltZW5zaW9ucyBzaG91bGQgYmUgb2YgdGhpcyB2YWx1ZS5cbiAqL1xuTGF5b3V0Q29uc3RhbnRzLkVNUFRZX0NPTVBPVU5EX05PREVfU0laRSA9IDQwO1xuXG4vKlxuICogTWluaW11bSBsZW5ndGggdGhhdCBhbiBlZGdlIHNob3VsZCB0YWtlIGR1cmluZyBsYXlvdXRcbiAqL1xuTGF5b3V0Q29uc3RhbnRzLk1JTl9FREdFX0xFTkdUSCA9IDE7XG5cbi8qXG4gKiBXb3JsZCBib3VuZGFyaWVzIHRoYXQgbGF5b3V0IG9wZXJhdGVzIG9uXG4gKi9cbkxheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWSA9IDEwMDAwMDA7XG5cbi8qXG4gKiBXb3JsZCBib3VuZGFyaWVzIHRoYXQgcmFuZG9tIHBvc2l0aW9uaW5nIGNhbiBiZSBwZXJmb3JtZWQgd2l0aFxuICovXG5MYXlvdXRDb25zdGFudHMuSU5JVElBTF9XT1JMRF9CT1VOREFSWSA9IExheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWSAvIDEwMDA7XG5cbi8qXG4gKiBDb29yZGluYXRlcyBvZiB0aGUgd29ybGQgY2VudGVyXG4gKi9cbkxheW91dENvbnN0YW50cy5XT1JMRF9DRU5URVJfWCA9IDEyMDA7XG5MYXlvdXRDb25zdGFudHMuV09STERfQ0VOVEVSX1kgPSA5MDA7XG5cbm1vZHVsZS5leHBvcnRzID0gTGF5b3V0Q29uc3RhbnRzO1xuIiwidmFyIE9yZ2FuaXphdGlvbiA9IHJlcXVpcmUoJy4vT3JnYW5pemF0aW9uJyk7XHJcblxyXG5mdW5jdGlvbiBNZW1iZXJQYWNrKGNoaWxkR3JhcGgpXHJcbntcclxuICAgIHZhciBtZW1iZXJzID0gW10gLypBcnJheUxpc3Q8U2JnblBETm9kZT4qLztcclxuICAgIG1lbWJlcnMuY29uY2F0KGNoaWxkR3JhcGguZ2V0Tm9kZXMoKSk7XHJcbiAgICBcclxuICAgIHZhciBvcmcgPSBuZXcgT3JnYW5pemF0aW9uKCk7XHJcblxyXG4gICAgdGhpcy5sYXlvdXQoKTtcclxuXHJcbiAgICB2YXIgbm9kZXMgPSBbXTtcclxuXHJcbiAgICBmb3IgKHZhciBpPTA7IGk8Y2hpbGRHcmFwaC5nZXROb2RlcygpLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIG5vZGVzW2ldID0gY2hpbGRHcmFwaC5nZXROb2RlcygpW2ldO1xyXG4gICAgfVxyXG59XHJcblxyXG5NZW1iZXJQYWNrLnByb3RvdHlwZS5sYXlvdXQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgY29tcGFyID0gW10gLypuZXcgQ29tcGFyYWJsZU5vZGVbbWVtYmVycy5zaXplKCldKi87XHJcblxyXG4gICAgdmFyIGk9MDtcclxuICAgIGZvciAodmFyIGo9MDsgajx0aGlzLm1lbWJlcnMubGVuZ3RoOyBqKyspXHJcbiAgICB7XHJcbiAgICAgICAgY29tcGFyW2krK10gPSBtZW1iZXJzW2pdO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICBjb21wYXIuc29ydCh0aGlzLmNvbXBhcmVOb2Rlcyk7XHJcbiAgICB0aGlzLm1lbWJlcnMgPSBbXTtcclxuICAgIFxyXG4gICAgZm9yICh2YXIgaj0wOyBqPGNvbXBhci5sZW5ndGg7IGorKylcclxuICAgIHtcclxuICAgICAgICB0aGlzLm1lbWJlcnMucHVzaChjb21wYXJbal0uZ2V0Tm9kZSgpKTtcclxuICAgIH1cclxuICAgIFxyXG4gICAgZm9yICh2YXIgaj0wOyBqPHRoaXMubWVtYmVycy5sZW5ndGg7IGorKylcclxuICAgIHtcclxuICAgICAgICB0aGlzLm9yZy5pbnNlcnROb2RlKG1lbWJlcnNbal0pO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIENvbXBhY3Rpb24gYyA9IG5ldyBDb21wYWN0aW9uKFxyXG4gICAgLy8gKEFycmF5TGlzdDxTYmduUEROb2RlPikgbWVtYmVycyk7XHJcbiAgICAvLyBjLnBlcmZvcm0oKTtcclxuXHJcbn07XHJcblxyXG5NZW1iZXJQYWNrLnByb3RvdHlwZS5nZXRXaWR0aCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHJldHVybiB0aGlzLm9yZy5nZXRXaWR0aCgpO1xyXG59O1xyXG5cclxuTWVtYmVyUGFjay5wcm90b3R5cGUuZ2V0SGVpZ2h0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMub3JnLmdldEhlaWdodCgpO1xyXG59O1xyXG5cclxuTWVtYmVyUGFjay5wcm90b3R5cGUuYWRqdXN0TG9jYXRpb25zID0gZnVuY3Rpb24gKHgsIHkpXHJcbntcclxuICAgIHRoaXMub3JnLmFkanVzdExvY2F0aW9ucyh4LCB5KTtcclxufTtcclxuXHJcbk1lbWJlclBhY2sucHJvdG90eXBlLmdldE1lbWJlcnMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy5tZW1iZXJzO1xyXG59O1xyXG5cclxuTWVtYmVyUGFjay5wcm90b3R5cGUuY29tcGFyZU5vZGVzID0gZnVuY3Rpb24gKG5vZGVBLCBub2RlQilcclxue1xyXG4gICAgdmFyIGFTaXplID0gbm9kZUEuZ2V0V2lkdGgoKSAqIG5vZGVBLmdldEhlaWdodCgpO1xyXG4gICAgdmFyIGJTaXplID0gbm9kZUIuZ2V0V2lkdGgoKSAqIG5vZGVCLmdldEhlaWdodCgpO1xyXG4gICAgXHJcbiAgICBpZiAoYVNpemUgPiBiU2l6ZSlcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gMTtcclxuICAgIH1cclxuICAgIGVsc2UgaWYgKGFTaXplIDwgYlNpemUpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIC0xO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiAwO1xyXG4gICAgfVxyXG4gICAgcmV0dXJuIHRoaXMubWVtYmVycztcclxufTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gTWVtYmVyUGFjazsiLCJ2YXIgU2JnblBEQ29uc3RhbnRzID0gcmVxdWlyZSgnLi9TYmduUERDb25zdGFudHMnKTtcclxuXHJcbi8qKlxyXG4qIENyZWF0ZXMgYSBjb250YWluZXIgd2hvc2Ugd2lkdGggYW5kIGhlaWdodCBpcyBvbmx5IHRoZSBtYXJnaW5zXHJcbiovXHJcbmZ1bmN0aW9uIE9yZ2FuaXphdGlvbigpXHJcbntcclxuICAgIHRoaXMud2lkdGggPSBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fTUFSR0lOICogMjtcclxuICAgIHRoaXMuaGVpZ2h0ID0gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTiAqIDIgO1xyXG5cclxuICAgIHRoaXMucm93V2lkdGggPSBbXSAvKm5ldyBBcnJheUxpc3Q8RG91YmxlPigpKi87XHJcbiAgICB0aGlzLnJvd0hlaWdodCA9IFtdIC8qbmV3IEFycmF5TGlzdDxEb3VibGU+KCkqLztcclxuXHJcbiAgICB0aGlzLnJvd3MgPSBbW11dOy8qbmV3IEFycmF5TGlzdDxMaW5rZWRMaXN0PFNiZ25QRE5vZGU+PigpOyovXHJcbn1cclxuXHJcbk9yZ2FuaXphdGlvbi5wcm90b3R5cGUuZ2V0V2lkdGggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICBzaGlmdFRvTGFzdFJvdygpO1xyXG4gICAgcmV0dXJuIHRoaXMud2lkdGg7XHJcbn07XHJcblxyXG5Pcmdhbml6YXRpb24ucHJvdG90eXBlLmdldEhlaWdodCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHJldHVybiB0aGlzLmhlaWdodDtcclxufTtcclxuXHJcbi8qKlxyXG4qIFNjYW5zIHRoZSByb3dXaWR0aCBhcnJheSBsaXN0IGFuZCByZXR1cm5zIHRoZSBpbmRleCBvZiB0aGUgcm93IHRoYXQgaGFzXHJcbiogdGhlIG1pbmltdW0gd2lkdGguXHJcbiovXHJcbk9yZ2FuaXphdGlvbi5wcm90b3R5cGUuZ2V0U2hvcnRlc3RSb3dJbmRleCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciByID0gLTE7XHJcbiAgICB2YXIgbWluID0gTnVtYmVyLk1BWF9WQUxVRTtcclxuICAgIFxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCB0aGlzLnJvd3MubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKHRoaXMucm93V2lkdGhbaV0gPCBtaW4pXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByID0gaTtcclxuICAgICAgICAgICAgbWluID0gdGhpcy5yb3dXaWR0aFtpXTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICByZXR1cm4gcjtcclxufTtcclxuXHJcbi8qKlxyXG4qIFNjYW5zIHRoZSByb3dXaWR0aCBhcnJheSBsaXN0IGFuZCByZXR1cm5zIHRoZSBpbmRleCBvZiB0aGUgcm93IHRoYXQgaGFzXHJcbiogdGhlIG1heGltdW0gd2lkdGguXHJcbiovXHJcbk9yZ2FuaXphdGlvbi5wcm90b3R5cGUuZ2V0TG9uZ2VzdFJvd0luZGV4ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIHIgPSAtMTtcclxuICAgIHZhciBtYXggPSBOdW1iZXIuTUFYX1ZBTFVFO1xyXG5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy5yb3dzLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIGlmICh0aGlzLnJvd1dpZHRoW2ldID4gbWF4KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgciA9IGk7XHJcbiAgICAgICAgICAgIG1heCA9IHRoaXMucm93V2lkdGhbaV07XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgcmV0dXJuIHI7XHJcbn07XHJcblxyXG5Pcmdhbml6YXRpb24ucHJvdG90eXBlLmluc2VydE5vZGUgPSBmdW5jdGlvbiAobm9kZSlcclxue1xyXG4gICAgaWYgKHRoaXMucm93cy5sZW5ndGggPD0gMClcclxuICAgIHtcclxuICAgICAgICBpbnNlcnROb2RlVG9Sb3cobm9kZSwgMCk7XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmIChjYW5BZGRIb3Jpem9udGFsKG5vZGUuZ2V0V2lkdGgoKSwgbm9kZS5nZXRIZWlnaHQoKSkpXHJcbiAgICB7XHJcbiAgICAgICAgaW5zZXJ0Tm9kZVRvUm93KG5vZGUsIGdldFNob3J0ZXN0Um93SW5kZXgoKSk7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgaW5zZXJ0Tm9kZVRvUm93KG5vZGUsIHRoaXMucm93cy5sZW5ndGgpO1xyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgcGVyZm9ybXMgdGlsaW5nLiBJZiBhIG5ldyByb3cgaXMgbmVlZGVkLCBpdCBjcmVhdGVzIHRoZSByb3dcclxuKiBhbmQgcGxhY2VzIHRoZSBuZXcgbm9kZSB0aGVyZS4gT3RoZXJ3aXNlLCBpdCBwbGFjZXMgdGhlIG5vZGUgdG8gdGhlIGVuZFxyXG4qIG9mIHRoZSBnaXZlbiByb3cuXHJcbiogXHJcbiogQHBhcmFtIG5vZGVcclxuKiBAcGFyYW0gcm93SW5kZXhcclxuKi9cclxuT3JnYW5pemF0aW9uLnByb3RvdHlwZS5pbnNlcnROb2RlVG9Sb3cgPSBmdW5jdGlvbiAobm9kZSwgcm93SW5kZXgpXHJcbntcclxuICAgIC8vIEFkZCBuZXcgcm93IGlmIG5lZWRlZFxyXG4gICAgaWYgKHJvd0luZGV4ID09PSB0aGlzLnJvd3MubGVuZ3RoKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMucm93cy5wdXNoKFtdKTtcclxuXHJcbiAgICAgICAgdGhpcy5yb3dXaWR0aC5wdXNoKFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01JTl9XSURUSCk7XHJcbiAgICAgICAgdGhpcy5yb3dIZWlnaHQucHVzaCgwLjApO1xyXG5cclxuICAgICAgICAvLyBUT0RPOiBhc3NlcnQgcm93cy5zaXplKCkgPT0gcm93V2lkdGguc2l6ZSgpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFVwZGF0ZSByb3cgd2lkdGhcclxuICAgIHZhciB3ID0gdGhpcy5yb3dXaWR0aFtyb3dJbmRleF0gKyBub2RlLmdldFdpZHRoKCk7XHJcblxyXG4gICAgaWYgKHRoaXMucm93c1tyb3dJbmRleF0gPiAwKVxyXG4gICAge1xyXG4gICAgICAgIHcgKz0gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX0hPUklaT05UQUxfQlVGRkVSO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB0aGlzLnJvd1dpZHRoLnNldFtyb3dJbmRleF0gPSB3O1xyXG5cclxuICAgIC8vIFVwZGF0ZSBjb21wbGV4IHdpZHRoXHJcbiAgICBpZiAodGhpcy53aWR0aCA8IHcpXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy53aWR0aCA9IHc7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gVXBkYXRlIGhlaWdodFxyXG4gICAgdmFyIGggPSBub2RlLmdldEhlaWdodCgpO1xyXG4gICAgaWYgKHJvd0luZGV4ID4gMClcclxuICAgIHtcclxuICAgICAgICBoICs9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVI7XHJcbiAgICB9XHJcbiAgICBcclxuICAgIHZhciBleHRyYUhlaWdodCA9IDA7XHJcbiAgICBpZiAoaCA+IHRoaXMucm93SGVpZ2h0W3Jvd0luZGV4XSlcclxuICAgIHtcclxuICAgICAgICBleHRyYUhlaWdodCA9IHRoaXMucm93SGVpZ2h0W3Jvd0luZGV4XTtcclxuICAgICAgICB0aGlzLnJvd0hlaWdodFtyb3dJbmRleF0gPSBoO1xyXG4gICAgICAgIGV4dHJhSGVpZ2h0ID0gdGhpcy5yb3dIZWlnaHRbcm93SW5kZXhdIC0gZXh0cmFIZWlnaHQ7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5oZWlnaHQgKz0gZXh0cmFIZWlnaHQ7XHJcblxyXG4gICAgLy8gSW5zZXJ0IG5vZGVcclxuICAgIHRoaXMucm93c1tyb3dJbmRleF0ucHVzaChub2RlKTtcclxufTtcclxuXHJcbi8qKlxyXG4qIElmIG1vdmluZyB0aGUgbGFzdCBub2RlIGZyb20gdGhlIGxvbmdlc3Qgcm93IGFuZCBhZGRpbmcgaXQgdG8gdGhlIGxhc3RcclxuKiByb3cgbWFrZXMgdGhlIGJvdW5kaW5nIGJveCBzbWFsbGVyLCBkbyBpdC5cclxuKi9cclxuT3JnYW5pemF0aW9uLnByb3RvdHlwZS5zaGlmdFRvTGFzdFJvdyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBsb25nZXN0ID0gdGhpcy5nZXRMb25nZXN0Um93SW5kZXgoKTtcclxuICAgIHZhciBsYXN0ID0gdGhpcy5yb3dXaWR0aC5sZW5ndGggLSAxO1xyXG4gICAgdmFyIHJvdyA9IHJvd3NbbG9uZ2VzdF07IC8qTGlua2VkTGlzdDxTYmduUEROb2RlPiovXHJcbiAgICB2YXIgbm9kZSA9IHJvd1socm93Lmxlbmd0aC0xKV07XHJcblxyXG4gICAgdmFyIGRpZmYgPSBub2RlLmdldFdpZHRoKCkgKyBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fSE9SSVpPTlRBTF9CVUZGRVI7XHJcblxyXG4gICAgaWYgKHRoaXMud2lkdGggLSB0aGlzLnJvd1dpZHRoWyh0aGlzLnJvd1dpZHRoLmxlbmd0aC0xKV0gPiBkaWZmICYmIFxyXG4gICAgICAgIHRoaXMucm93SGVpZ2h0Wyh0aGlzLnJvd0hlaWdodC5sZW5ndGgtMSldID4gbm9kZS5nZXRIZWlnaHQoKSlcclxuICAgIHtcclxuICAgICAgICByb3cucG9wKCk7XHJcbiAgICAgICAgdGhpcy5yb3dzW3RoaXMucm93cy5sZW5ndGhdLnB1c2gobm9kZSk7XHJcbiAgICAgICAgdGhpcy5yb3dXaWR0aFtsb25nZXN0XSA9IHRoaXMucm93V2lkdGhbbG9uZ2VzdF0gLSBkaWZmO1xyXG4gICAgICAgIHRoaXMucm93V2lkdGhbbGFzdF0gPSB0aGlzLnJvd1dpZHRoW2xhc3RdICsgZGlmZjtcclxuXHJcbiAgICAgICAgdGhpcy53aWR0aCA9IHRoaXMucm93V2lkdGhbdGhpcy5nZXRMb25nZXN0Um93SW5kZXgoKV07XHJcblxyXG4gICAgICAgIC8vIFVwZGF0ZSBoZWlnaHQgb2YgdGhlIG9yZ2FuaXphdGlvblxyXG4gICAgICAgIHZhciBtYXhIZWlnaHQgPSBOdW1iZXIuTUlOX1ZBTFVFO1xyXG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgcm93Lmxlbmd0aDsgaSsrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHJvd1tpXS5nZXRIZWlnaHQoKSA+IG1heEhlaWdodClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgbWF4SGVpZ2h0ID0gcm93W2ldLmdldEhlaWdodCgpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGlmIChsb25nZXN0ID4gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG1heEhlaWdodCArPSBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fVkVSVElDQUxfQlVGRkVSO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgdmFyIHByZXZUb3RhbCA9IHRoaXMucm93SGVpZ2h0W2xvbmdlc3RdICsgdGhpcy5yb3dIZWlnaHRbbGFzdF07XHJcblxyXG4gICAgICAgIHRoaXMucm93SGVpZ2h0W2xvbmdlc3RdID0gbWF4SGVpZ2h0O1xyXG4gICAgICAgIGlmICh0aGlzLnJvd0hlaWdodFtsYXN0XSA8IFxyXG4gICAgICAgICAgICBub2RlLmdldEhlaWdodCgpICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX1ZFUlRJQ0FMX0JVRkZFUilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucm93SGVpZ2h0W2xhc3RdID1cclxuICAgICAgICAgICAgICAgICAgICBub2RlLmdldEhlaWdodCgpICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX1ZFUlRJQ0FMX0JVRkZFUjtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHZhciBmaW5hbFRvdGFsID0gdGhpcy5yb3dIZWlnaHRbbG9uZ2VzdF0gKyB0aGlzLnJvd0hlaWdodFtsYXN0XTtcclxuICAgICAgICB0aGlzLmhlaWdodCArPSAoZmluYWxUb3RhbCAtIHByZXZUb3RhbCk7XHJcblxyXG4gICAgICAgIHRoaXMuc2hpZnRUb0xhc3RSb3coKTtcclxuICAgIH1cclxufTtcclxuXHJcbk9yZ2FuaXphdGlvbi5wcm90b3R5cGUuY2FuQWRkSG9yaXpvbnRhbCA9IGZ1bmN0aW9uIChleHRyYVdpZHRoLCBleHRyYUhlaWdodClcclxue1xyXG4gICAgdmFyIHNyaSA9IHRoaXMuZ2V0U2hvcnRlc3RSb3dJbmRleCgpO1xyXG5cclxuICAgIGlmIChzcmkgPCAwKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB2YXIgbWluID0gdGhpcy5yb3dXaWR0aFtzcmldO1xyXG4gICAgdmFyIGhEaWZmID0gMDtcclxuICAgIGlmICh0aGlzLnJvd0hlaWdodFtzcmldIDwgZXh0cmFIZWlnaHQpXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKHNyaSA+IDApXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBoRGlmZiA9IGV4dHJhSGVpZ2h0ICsgXHJcbiAgICAgICAgICAgICAgICAgICAgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX1ZFUlRJQ0FMX0JVRkZFUiAtIFxyXG4gICAgICAgICAgICAgICAgICAgIHRoaXMucm93SGVpZ2h0W3NyaV07XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgaWYgKCh0aGlzLndpZHRoIC0gbWluKSA+PSBcclxuICAgICAgICAoZXh0cmFXaWR0aCArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUikpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuICh0aGlzLmhlaWdodCArIGhEaWZmKSA+IFxyXG4gICAgICAgICAgIChtaW4gKyBleHRyYVdpZHRoICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX0hPUklaT05UQUxfQlVGRkVSKTtcclxufTtcclxuXHJcbk9yZ2FuaXphdGlvbi5wcm90b3R5cGUuYWRqdXN0TG9jYXRpb25zID0gZnVuY3Rpb24gKHgsIHkpXHJcbntcclxuICAgIHggKz0gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTjtcclxuICAgIHkgKz0gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTjtcclxuXHJcbiAgICB2YXIgbGVmdCA9IHg7XHJcblxyXG4gICAgZm9yICh2YXIgaT0wOyBpPHRoaXMucm93cy5sZW5ndGg7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgcm93ID0gdGhpcy5yb3dzW2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIHggPSBsZWZ0O1xyXG4gICAgICAgIHZhciBtYXhIZWlnaHQgPSAwO1xyXG4gICAgICAgIGZvciAodmFyIGo9MDsgajxyb3cubGVuZ3RoOyBqKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgbm9kZSA9IHJvd1tqXTtcclxuICAgICAgICAgICAgbm9kZS5zZXRMb2NhdGlvbih4LCB5KTtcclxuXHJcbiAgICAgICAgICAgIHggKz0gKG5vZGUuZ2V0V2lkdGgoKSArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUik7XHJcblxyXG4gICAgICAgICAgICBpZiAobm9kZS5nZXRIZWlnaHQoKSA+IG1heEhlaWdodClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgbWF4SGVpZ2h0ID0gbm9kZS5nZXRIZWlnaHQoKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgeSArPSAobWF4SGVpZ2h0ICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX1ZFUlRJQ0FMX0JVRkZFUik7XHJcbiAgICB9XHJcbn07XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IE9yZ2FuaXphdGlvbjtcclxuIiwiLypcbiAqVGhpcyBjbGFzcyBpcyB0aGUgamF2YXNjcmlwdCBpbXBsZW1lbnRhdGlvbiBvZiB0aGUgUG9pbnQuamF2YSBjbGFzcyBpbiBqZGtcbiAqL1xuZnVuY3Rpb24gUG9pbnQoeCwgeSwgcCkge1xuICB0aGlzLnggPSBudWxsO1xuICB0aGlzLnkgPSBudWxsO1xuICBpZiAoeCA9PSBudWxsICYmIHkgPT0gbnVsbCAmJiBwID09IG51bGwpIHtcbiAgICB0aGlzLnggPSAwO1xuICAgIHRoaXMueSA9IDA7XG4gIH1cbiAgZWxzZSBpZiAodHlwZW9mIHggPT0gJ251bWJlcicgJiYgdHlwZW9mIHkgPT0gJ251bWJlcicgJiYgcCA9PSBudWxsKSB7XG4gICAgdGhpcy54ID0geDtcbiAgICB0aGlzLnkgPSB5O1xuICB9XG4gIGVsc2UgaWYgKHguY29uc3RydWN0b3IubmFtZSA9PSAnUG9pbnQnICYmIHkgPT0gbnVsbCAmJiBwID09IG51bGwpIHtcbiAgICBwID0geDtcbiAgICB0aGlzLnggPSBwLng7XG4gICAgdGhpcy55ID0gcC55O1xuICB9XG59XG5cblBvaW50LnByb3RvdHlwZS5nZXRYID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gdGhpcy54O1xufVxuXG5Qb2ludC5wcm90b3R5cGUuZ2V0WSA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIHRoaXMueTtcbn1cblxuUG9pbnQucHJvdG90eXBlLmdldExvY2F0aW9uID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gbmV3IFBvaW50KHRoaXMueCwgdGhpcy55KTtcbn1cblxuUG9pbnQucHJvdG90eXBlLnNldExvY2F0aW9uID0gZnVuY3Rpb24gKHgsIHksIHApIHtcbiAgaWYgKHguY29uc3RydWN0b3IubmFtZSA9PSAnUG9pbnQnICYmIHkgPT0gbnVsbCAmJiBwID09IG51bGwpIHtcbiAgICBwID0geDtcbiAgICB0aGlzLnNldExvY2F0aW9uKHAueCwgcC55KTtcbiAgfVxuICBlbHNlIGlmICh0eXBlb2YgeCA9PSAnbnVtYmVyJyAmJiB0eXBlb2YgeSA9PSAnbnVtYmVyJyAmJiBwID09IG51bGwpIHtcbiAgICAvL2lmIGJvdGggcGFyYW1ldGVycyBhcmUgaW50ZWdlciBqdXN0IG1vdmUgKHgseSkgbG9jYXRpb25cbiAgICBpZiAocGFyc2VJbnQoeCkgPT0geCAmJiBwYXJzZUludCh5KSA9PSB5KSB7XG4gICAgICB0aGlzLm1vdmUoeCwgeSk7XG4gICAgfVxuICAgIGVsc2Uge1xuICAgICAgdGhpcy54ID0gTWF0aC5mbG9vcih4ICsgMC41KTtcbiAgICAgIHRoaXMueSA9IE1hdGguZmxvb3IoeSArIDAuNSk7XG4gICAgfVxuICB9XG59XG5cblBvaW50LnByb3RvdHlwZS5tb3ZlID0gZnVuY3Rpb24gKHgsIHkpIHtcbiAgdGhpcy54ID0geDtcbiAgdGhpcy55ID0geTtcbn1cblxuUG9pbnQucHJvdG90eXBlLnRyYW5zbGF0ZSA9IGZ1bmN0aW9uIChkeCwgZHkpIHtcbiAgdGhpcy54ICs9IGR4O1xuICB0aGlzLnkgKz0gZHk7XG59XG5cblBvaW50LnByb3RvdHlwZS5lcXVhbHMgPSBmdW5jdGlvbiAob2JqKSB7XG4gIGlmIChvYmouY29uc3RydWN0b3IubmFtZSA9PSBcIlBvaW50XCIpIHtcbiAgICB2YXIgcHQgPSBvYmo7XG4gICAgcmV0dXJuICh0aGlzLnggPT0gcHQueCkgJiYgKHRoaXMueSA9PSBwdC55KTtcbiAgfVxuICByZXR1cm4gdGhpcyA9PSBvYmo7XG59XG5cblBvaW50LnByb3RvdHlwZS50b1N0cmluZyA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIG5ldyBQb2ludCgpLmNvbnN0cnVjdG9yLm5hbWUgKyBcIlt4PVwiICsgdGhpcy54ICsgXCIseT1cIiArIHRoaXMueSArIFwiXVwiO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IFBvaW50O1xuIiwiZnVuY3Rpb24gUG9pbnREKHgsIHkpIHtcbiAgaWYgKHggPT0gbnVsbCAmJiB5ID09IG51bGwpIHtcbiAgICB0aGlzLnggPSAwO1xuICAgIHRoaXMueSA9IDA7XG4gIH0gZWxzZSB7XG4gICAgdGhpcy54ID0geDtcbiAgICB0aGlzLnkgPSB5O1xuICB9XG59XG5cblBvaW50RC5wcm90b3R5cGUuZ2V0WCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLng7XG59O1xuXG5Qb2ludEQucHJvdG90eXBlLmdldFkgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy55O1xufTtcblxuUG9pbnRELnByb3RvdHlwZS5zZXRYID0gZnVuY3Rpb24gKHgpXG57XG4gIHRoaXMueCA9IHg7XG59O1xuXG5Qb2ludEQucHJvdG90eXBlLnNldFkgPSBmdW5jdGlvbiAoeSlcbntcbiAgdGhpcy55ID0geTtcbn07XG5cblBvaW50RC5wcm90b3R5cGUuZ2V0RGlmZmVyZW5jZSA9IGZ1bmN0aW9uIChwdClcbntcbiAgcmV0dXJuIG5ldyBEaW1lbnNpb25EKHRoaXMueCAtIHB0LngsIHRoaXMueSAtIHB0LnkpO1xufTtcblxuUG9pbnRELnByb3RvdHlwZS5nZXRDb3B5ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIG5ldyBQb2ludEQodGhpcy54LCB0aGlzLnkpO1xufTtcblxuUG9pbnRELnByb3RvdHlwZS50cmFuc2xhdGUgPSBmdW5jdGlvbiAoZGltKVxue1xuICB0aGlzLnggKz0gZGltLndpZHRoO1xuICB0aGlzLnkgKz0gZGltLmhlaWdodDtcbiAgcmV0dXJuIHRoaXM7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IFBvaW50RDtcbiIsImZ1bmN0aW9uIFBvbHlvbWlubygpIFxue1xuICAgIC8vIHRoZSBudW1iZXIgb2YgY2VsbHNcbiAgICB0aGlzLmwgPSAwO1xuXG4gICAgLy90aGUgcmVzdWx0aW5nIHBsYWNlbWVudCBjb29yZGluYXRlc1xuICAgIHRoaXMueCA9IDA7XG4gICAgdGhpcy55ID0gMDtcblxuICAgIHRoaXMubGFiZWwgPSBcIlwiO1xuXG4gICAgLy8gcG9seW9taW5vIGNlbGxzXG4gICAgdGhpcy5jb29yZCA9IFtdO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IFBvbHlvbWlubzsiLCJ2YXIgUG9seW9taW5vUXVpY2tTb3J0ID0gcmVxdWlyZSgnLi9Qb2x5b21pbm9RdWlja1NvcnQnKTtcbnZhciBJbnRlZ2VyID0gcmVxdWlyZSgnLi9JbnRlZ2VyJyk7XG52YXIgUmVjdGFuZ2xlRCA9IHJlcXVpcmUoJy4vUmVjdGFuZ2xlRCcpO1xuXG5mdW5jdGlvbiBQb2x5b21pbm9QYWNraW5nKClcbntcbiAgICAvLyBQb2x5b21pbm8gYXJyYXlcbiAgIHRoaXMucG9seW9taW5vZXMgPSBbXTtcblxuICAgLy8gQm91bmRpbmcgcmVjdGFuZ2xlcyBvZiB0aGUgcG9seW9taW5vZXNcbiAgIHRoaXMucmVjdCA9IFtdO1xuXG4gICAvLyBUaGUgZ3JpZFxuICAgdGhpcy5ncmlkID0gW1tdXTtcblxuICAgLy8gQ2VudGVyIHBvaW50IG9mIHRoZSBncmlkXG4gICB0aGlzLmdjeCA9IDA7XG4gICB0aGlzLmdjeSA9IDA7XG5cbiAgIC8vIEdyaWQgc2l6ZVxuICAgdGhpcy5zaXplWCA9IDA7XG4gICB0aGlzLnNpemVZID0gMDtcblxuICAgLy8gVGhlIG51bWJlciBvZiBhbHJlYWR5IHBsYWNlZCBwb2x5b21pbm9lc1xuICAgdGhpcy5jdXJtaW5vID0gMDtcblxuICAgLy8gU3RvcmVzIHRoZSBvcmRlcmluZyBvZiB0aGUgcG9seW9taW5vZXNcbiAgIHRoaXMuaW5kID0gW107XG5cbiAgIC8vIFJhbmRvbSBnZW5lcmF0b3JcbiAgIC8vVE9ETzogUmFuZG9tIFJnZW47XG59XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBwZXJmb3JtcyBwb2x5b21pbm8gcGFja2luZy5cbiovXG5Qb2x5b21pbm9QYWNraW5nLnByb3RvdHlwZS5wYWNrID0gZnVuY3Rpb24gKHBtLCBwY291bnQpXG57XG4gICAgdGhpcy5wb2x5b21pbm9lcyA9IHBtO1xuICAgIHRoaXMucmVjdCA9IFtdO1xuXG4gICAgLy8gbWFrZSB0aGUgaW5pdGlhbCBncmlkXG4gICAgdGhpcy5tYWtlR3JpZCgxMDAsIDEwMCwgMCk7XG5cbiAgICAvLyBtYWtlIHRoZSByYW5kb20gcGVybXV0YXRpb24gb2YgcG9seW9taW5vIGNlbGxzIGFuZFxuICAgIC8vIGNhbGN1bGF0ZSB0aGUgYm91bmRpbmcgcmVjdGFuZ2xlcy5cbiAgICAvLyBUT0RPOiBSZ2VuID0gbmV3IFJhbmRvbSgxKTtcbiAgICBmb3IgKHZhciBrID0gMDsgayA8IHBjb3VudDsgaysrKVxuICAgIHtcbiAgICAgICAgdGhpcy5SYW5kb21pemVNaW5vKGspO1xuICAgIH1cbiAgICBcbiAgICAvLyBvcmRlciB0aGUgcG9seW9taW5vZXMgaW4gaW5jcmVhc2luZyBzaXplXG4gICAgdmFyIGtleSA9IFtdO1xuICAgIHRoaXMuaW5kID0gW107XG5cbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHBjb3VudDsgaSsrKVxuICAgIHtcbiAgICAgICAga2V5W2ldID0gLSh0aGlzLnJlY3RbaV0uZ2V0TWF4WCgpIC0gdGhpcy5yZWN0W2ldLmdldE1pblgoKSlcbiAgICAgICAgICAgICAgICAgICAgICAgIC0gKHRoaXMucmVjdFtpXS5nZXRNYXhZKCkgLSB0aGlzLnJlY3RbaV0uZ2V0TWluWSgpKTtcbiAgICB9XG4gICAgdmFyIHFzb3J0ID0gbmV3IFBvbHlvbWlub1F1aWNrU29ydCgpO1xuICAgIHFzb3J0LnNvcnQocGNvdW50LCBrZXksIHRoaXMuaW5kKTtcblxuICAgIC8vIHBsYWNlIG9uZSBieSBvbmUgc3RhcnRpbmcgZnJvbSB0aGUgbGFyZ2VzdFxuICAgIGZvciAodGhpcy5jdXJtaW5vID0gMDsgdGhpcy5jdXJtaW5vIDwgcGNvdW50OyB0aGlzLmN1cm1pbm8rKylcbiAgICB7XG4gICAgICAgIHB1dE1pbm8odGhpcy5pbmRbdGhpcy5jdXJtaW5vXSk7XG4gICAgfVxufTtcblxuLyoqXG4qIFRoaXMgY3JlYXRlcyB0aGUgZ3JpZCBvZiBnaXZlbiBkaW1lbnNpb25zIGFuZCBmaWxscyBpdCB3aXRoIHRoZSBhbHJlYWR5XG4qIHBsYWNlZCBwb2x5b21pbm9lcy5cbiovXG5Qb2x5b21pbm9QYWNraW5nLnByb3RvdHlwZS5tYWtlR3JpZCA9IGZ1bmN0aW9uIChkaW14LCBkaW15LCBtTilcbntcbiAgICB2YXIgaTtcblxuICAgIC8vIGFsbG9jYXRlIHRoZSBncmlkXG4gICAgLyogVE9ETzogV2UgZG9uJ3QgbmVlZCB0aGlzISBcbiAgICB0aGlzLmdyaWQgPSBbW11dO1xuICAgIGZvciAoaSA9IDA7IGkgPCBkaW15OyBpKyspXG4gICAge1xuICAgICAgICB0aGlzLmdyaWRbaV0gPSBuZXcgYnl0ZVtkaW14XTtcbiAgICB9Ki9cblxuICAgIHZhciBkeCA9IGRpbXggLyAyIC0gdGhpcy5nY3g7XG4gICAgdmFyIGR5ID0gZGlteSAvIDIgLSB0aGlzLmdjeTtcbiAgICB0aGlzLmdjeCA9IGRpbXggLyAyO1xuICAgIHRoaXMuZ2N5ID0gZGlteSAvIDI7XG4gICAgdGhpcy5zaXplWCA9IGRpbXg7XG4gICAgdGhpcy5zaXplWSA9IGRpbXk7XG5cbiAgICAvLyBtYXJrIHRoZSBwb3NpdGlvbnMgb2NjdXBpZWQgd2l0aCB0aGUgYWxyZWFkeSBwbGFjZWRcbiAgICAvLyBwb2x5b21pbm9lcy5cbiAgICBmb3IgKGkgPSAwOyBpIDwgbU47IGkrKylcbiAgICB7XG4gICAgICAgIHZhciBwID0gdGhpcy5wb2x5b21pbm9lc1t0aGlzLmluZFtpXV07XG4gICAgICAgIHAueCArPSBkeDtcbiAgICAgICAgcC55ICs9IGR5O1xuXG4gICAgICAgIGZvciAodmFyIGsgPSAwOyBrIDwgcC5sOyBrKyspXG4gICAgICAgIHtcbiAgICAgICAgICAgIHZhciB4eCA9IHAuY29vcmRba10uZ2V0WCgpICsgcC54O1xuICAgICAgICAgICAgdmFyIHl5ID0gcC5jb29yZFtrXS5nZXRZKCkgKyBwLnk7XG4gICAgICAgICAgICB0aGlzLmdyaWRbeXldW3h4XSA9IDE7XG4gICAgICAgIH1cbiAgICB9XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgY2hlY2tzIHdoZXRoZXIgcCBjYW4gYmUgcGxhY2VkIGluICh4LHkpLiBGb3IgZWFjaCBwb2x5b21pbm8sXG4qIGNoZWNrIGlmIHRoZSAoeCx5KSBpcyBvY2N1cGllZC9maXRzIGluIHRoZSBncmlkLlxuKi9cblBvbHlvbWlub1BhY2tpbmcucHJvdG90eXBlLklzRnJlZVBsYWNlID0gZnVuY3Rpb24gKHgsIHksIHApXG57XG4gICAgZm9yICh2YXIgayA9IDA7IGsgPCBwLmw7IGsrKylcbiAgICB7XG4gICAgICAgIHZhciB4eCA9IHAuY29vcmRba10uZ2V0WCgpICsgeDtcbiAgICAgICAgdmFyIHl5ID0gcC5jb29yZFtrXS5nZXRZKCkgKyB5O1xuICAgICAgICBcbiAgICAgICAgLy8gcmV0dXJuIGZhbHNlIGlmIHRoZSBwb2x5b21pbm8gZ29lcyBvdXRzaWRlIHRoZSBncmlkXG4gICAgICAgIGlmICh4eCA8IDAgfHwgeXkgPCAwIHx8IHh4ID49IHRoaXMuc2l6ZVggfHwgeXkgPj0gdGhpcy5zaXplWSlcbiAgICAgICAge1xuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG4gICAgICAgIFxuICAgICAgICAvLyBvciB0aGUgcG9zaXRpb24gaXMgb2NjdXBpZWRcbiAgICAgICAgaWYgKGdyaWRbeXldW3h4XSAhPT0gMClcbiAgICAgICAge1xuICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xuICAgICAgICB9XG4gICAgfVxuXG4gICAgLy8gcmVtZW1iZXIgdGhlIHBvc2l0aW9uXG4gICAgcC54ID0geDtcbiAgICBwLnkgPSB5O1xuICAgIHJldHVybiB0cnVlO1xufTtcblxuLyoqXG4qIFRoaXMgdHJpZXMgdG8gZmluZCBhIGZyZWUgcGxhY2UgaW4gdGhlIGdyaWQuIFRoZSBmdW5jdGlvbiByZXR1cm5zIHRydWUgaWZcbiogdGhlIHBsYWNlbWVudCBpcyBzdWNjZXNzZnVsLlxuKi9cblBvbHlvbWlub1BhY2tpbmcucHJvdG90eXBlLnRyeVBsYWNpbmcgPSBmdW5jdGlvbiAocGkpXG57XG4gICAgdmFyIHAgPSB0aGlzLnBvbHlvbWlub2VzW3BpXTtcblxuICAgIHZhciBjeCA9IHRoaXMuZ2N4IC0gKHRoaXMucmVjdFtwaV0uZ2V0TWF4WCgpICsgdGhpcy5yZWN0W3BpXS5nZXRNaW5YKCkpIC8gMjtcbiAgICB2YXIgY3kgPSB0aGlzLmdjeSAtICh0aGlzLnJlY3RbcGldLmdldE1heFkoKSArIHRoaXMucmVjdFtwaV0uZ2V0TWluWSgpKSAvIDI7XG5cbiAgICAvLyBzZWUgaWYgdGhlIGNlbnRlciBwb2ludCBpcyBub3Qgb2NjdXBpZWRcbiAgICBpZiAodGhpcy5Jc0ZyZWVQbGFjZShjeCwgY3ksIHApKVxuICAgIHtcbiAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgfVxuXG4gICAgLy8gdHJ5IHBsYWNpbmcgaW4gdGhlIGluY3JlYXNpbmcgZGlzdGFuY2UgZnJvbSB0aGUgY2VudGVyXG4gICAgZm9yICh2YXIgZCA9IDE7IGQgPCB0aGlzLnNpemVYIC8gMjsgZCsrKVxuICAgIHtcbiAgICAgICAgZm9yICh2YXIgaSA9IC1kOyBpIDwgZDsgaSsrKVxuICAgICAgICB7XG4gICAgICAgICAgICB2YXIgaTEgPSAoaSArIGQgKyAxKSAvIDIgKiAoKChpICYgMSkgPT09IDEpID8gMSA6IC0xKTtcbiAgICAgICAgICAgIGlmICh0aGlzLklzRnJlZVBsYWNlKC1kICsgY3gsIC1pMSArIGN5LCBwKSlcbiAgICAgICAgICAgICAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgICAgICAgICBpZiAodGhpcy5Jc0ZyZWVQbGFjZShkICsgY3gsIGkxICsgY3ksIHApKVxuICAgICAgICAgICAgICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgICAgICAgIGlmICh0aGlzLklzRnJlZVBsYWNlKGN4IC0gaTEsIGQgKyBjeSwgcCkpXG4gICAgICAgICAgICAgICAgICAgIHJldHVybiB0cnVlO1xuICAgICAgICAgICAgaWYgKHRoaXMuSXNGcmVlUGxhY2UoaTEgKyBjeCwgLWQgKyBjeSwgcCkpXG4gICAgICAgICAgICAgICAgICAgIHJldHVybiB0cnVlO1xuICAgICAgICB9XG4gICAgfVxuICAgIHJldHVybiBmYWxzZTtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBwbGFjZXMgdGhlIGdpdmVuIHBvbHlvbWluby4gVGhlIGdyaWQgaXMgZW5sYXJnZWQgaWZcbiogbmVjZXNzYXJ5LlxuKi9cblBvbHlvbWlub1BhY2tpbmcucHJvdG90eXBlLnB1dE1pbm8gPSBmdW5jdGlvbiAocGkpXG57XG4gICAgdmFyIHAgPSB0aGlzLnBvbHlvbWlub2VzW3BpXTtcblxuICAgIC8vIGlmIHRoZSBwb2x5b21pbm8gY2Fubm90IGJlIHBsYWNlZCBpbiB0aGUgY3VycmVudCBncmlkLFxuICAgIC8vIGVubGFyZ2UgaXQuXG4gICAgd2hpbGUgKCF0aGlzLnRyeVBsYWNpbmcocGkpKVxuICAgIHtcbiAgICAgICAgdGhpcy5zaXplWCArPSAxMDtcbiAgICAgICAgdGhpcy5zaXplWSArPSAxMDtcbiAgICAgICAgdGhpcy5tYWtlR3JpZCh0aGlzLnNpemVYLCB0aGlzLnNpemVZLCB0aGlzLmN1cm1pbm8pO1xuICAgIH1cblxuICAgIC8vIG1hcmsgdGhlIHBvc2l0aW9ucyBvY2N1cGllZFxuICAgIGZvciAodmFyIGsgPSAwOyBrIDwgcC5sOyBrKyspXG4gICAge1xuICAgICAgICB2YXIgeHggPSBwLmNvb3JkW2tdLmdldFgoKSArIHAueDtcbiAgICAgICAgdmFyIHl5ID0gcC5jb29yZFtrXS5nZXRZKCkgKyBwLnk7XG4gICAgICAgIHRoaXMuZ3JpZFt5eV1beHhdID0gMTtcbiAgICB9XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgbWFrZXMgYSByYW5kb20gcGVybXV0YXRpb24gb2YgcG9seW9taW5vIGNlbGxzIGFuZCBjYWxjdWxhdGVzXG4qIHRoZSBib3VuZGluZyByZWN0YW5nbGVzIG9mIHRoZSBwb2x5b21pbm9lcy5cbiovXG5Qb2x5b21pbm9QYWNraW5nLnByb3RvdHlwZS5SYW5kb21pemVNaW5vID0gZnVuY3Rpb24gKHBpKVxue1xuICAgIHZhciBwID0gcG9seW9taW5vZXNbcGldO1xuICAgIHZhciBpO1xuXG4gICAgLy8gbWFrZSB0aGUgcmFuZG9tIHBlcm11dGF0aW9uLiBUaGVvcmV0aWNhbGx5IGl0IHNwZWVkcyB1cCB0aGVcbiAgICAvLyBhbGdvcml0aG0gYSBsaXR0bGUuXG4gICAgZm9yIChpID0gMDsgaSA8IHAubDsgaSsrKVxuICAgIHtcbiAgICAgICAgdmFyIGkxID0gTWF0aC5yYW5kb20oKSAqIChwLmwgLSBpKSArIGk7XG4gICAgICAgIHZhciB0bXAgPSBwLmNvb3JkW2ldO1xuICAgICAgICBwLmNvb3JkW2ldID0gcC5jb29yZFtpMV07XG4gICAgICAgIHAuY29vcmRbaTFdID0gdG1wO1xuICAgIH1cblxuICAgIC8vIGNhbGN1bGF0ZSB0aGUgYm91bmRpbmcgcmVjdGFuZ2xlIG9mIHRoZSBwb2x5b21pbm9cbiAgICB0aGlzLnJlY3RbcGldID0gbmV3IFJlY3RhbmdsZUQoKTtcblxuICAgIHZhciBtaW5YID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gICAgdmFyIG1pblkgPSBJbnRlZ2VyLk1BWF9WQUxVRTtcbiAgICB2YXIgbWF4WCA9IEludGVnZXIuTUlOX1ZBTFVFO1xuICAgIHZhciBtYXhZID0gSW50ZWdlci5NSU5fVkFMVUU7XG4gICAgcC54ID0gcC55ID0gMDtcblxuICAgIGZvciAoaSA9IDA7IGkgPCBwLmw7IGkrKylcbiAgICB7XG4gICAgICAgIGlmIChwLmNvb3JkW2ldLmdldFgoKSA8IG1pblgpXG4gICAgICAgIHtcbiAgICAgICAgICAgIG1pblggPSBwLmNvb3JkW2ldLmdldFgoKTtcbiAgICAgICAgfVxuICAgICAgICBpZiAocC5jb29yZFtpXS5nZXRZKCkgPCBtaW5ZKVxuICAgICAgICB7XG4gICAgICAgICAgICBtaW5ZID0gcC5jb29yZFtpXS5nZXRZKCk7XG4gICAgICAgIH1cbiAgICAgICAgaWYgKHAuY29vcmRbaV0uZ2V0WCgpID4gbWF4WClcbiAgICAgICAge1xuICAgICAgICAgICAgbWF4WCA9IHAuY29vcmRbaV0uZ2V0WCgpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChwLmNvb3JkW2ldLmdldFkoKSA+IG1heFkpXG4gICAgICAgIHtcbiAgICAgICAgICAgIG1heFkgPSBwLmNvb3JkW2ldLmdldFkoKTtcbiAgICAgICAgfVxuICAgIH1cblxuICAgIHRoaXMucmVjdFtwaV0ueCA9IG1pblg7XG4gICAgdGhpcy5yZWN0W3BpXS55ID0gbWluWTtcbiAgICB0aGlzLnJlY3RbcGldLndpZHRoID0gbWF4WCAtIG1pblg7XG4gICAgdGhpcy5yZWN0W3BpXS5oZWlnaHQgPSBtYXhZIC0gbWluWTtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gUG9seW9taW5vUGFja2luZzsiLCJmdW5jdGlvbiBQb2x5b21pbm9RdWlja1NvcnQoKVxue1xuICAgIC8vIFRoaXMgdmFyaWFibGUgc3RvcmVzIHRoZSBudW1iZXIgb2YgZWxlbWVudHMgdG8gYmUgc29ydGVkXG4gICAgdGhpcy5uID0gMDtcblxuICAgIC8vIFRoaXMgYXJyYXkgc3RvcmVzIHRoZSBlbGVtZW50cyB0byBiZSBzb3J0ZWRcbiAgICB0aGlzLmtleSA9IFtdO1xuXG4gICAgLy8gVGhpcyBhcnJheSBpcyB1c2VkIGJ5IHN0YXRpYyBzb3J0ZXIgYW5kIHJlcHJlc2VudHMgdGhlIGN1cnJlbnRcbiAgICAvLyBvcmRlciBvZiB0aGUgaW5wdXQgYXJyYXlcbiAgICB0aGlzLmluZGV4ID0gW107XG59XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBpcyBhIHN0YXRpYyBzb3J0ZXIuXG4qICBQYXJhbWV0aGVyczpcbiogICAgICBuIC0gaXMgdGhlIG51bWJlciBvZiBkb3VibGVzIHRvIGJlIHNvcnRlZDtcbiogICAgICBrZXkgLSBpcyBhcnJheSBvZiBkb3VibGVzIHRvIGJlIHNvcnRlZC4gVGhlIG9yZGVyIG9mIGVsZW1lbnRzXG4qICAgICAgICAgIGluIHRoaXMgYXJyYXkgaXMgcHJlc2VydmVkO1xuKiAgICAgIGluZGV4IC0gaW4gdGhpcyBhcnJheSB0aGUgc29ydGVkIG9yZGVyIG9mIGVsZW1lbnRzIGlzXG4qICAgICAgICAgIHJldHVybmVkLiBpbmRleFtpXSA9IGogbWVhbnMgdGhhdCBpLXRoIGdyZWF0ZXN0IGVsZW1lbnRcbiogICAgICAgICAgaXMgai10aCBlbGVtZW50IG9mIGFycmF5IGtleS5cbiovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLnNvcnQgPSBmdW5jdGlvbiAobiwga2V5LCBpbmRleClcbntcbiAgIC8vIHN0b3JlIGFsbCBwYXJhbWV0aGVycyBhcyB0aGUgaW5zdGFuY2UgdmFyaWFibGVzXG4gICB0aGlzLm4gPSBuO1xuICAgdGhpcy5rZXkgPSBrZXk7XG4gICB0aGlzLmluZGV4ID0gaW5kZXg7XG5cbiAgIC8vIG1ha2UgdGhlIGluaXRpYWwgb3JkZXIgb2YgZWxlbWVudHNcbiAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy5uOyBpKyspXG4gICAgICAgdGhpcy5pbmRleFtpXSA9IGk7XG5cbiAgIC8vIGNhbGwgdGhlIHJlY3Vyc2l2ZSBzb3J0aW5nIG1ldGhvZCBvbiB0aGUgd2hvbGUgYXJyYXlcbiAgIHRoaXMucmVjU29ydFN0YXRpYygwLCB0aGlzLm4tMSk7XG5cbiAgIC8vIGRvIHRoZSBmaW5hbCBzb3J0IHdpdGggdGhlIGluc2VydGlvbiBzb3J0aW5nIGFsZ29yaXRobVxuICAgLy8gYmVjYXVzZSB0aGUgcmVjdXJzaXZlIHF1aWNrc29ydCByb3V0aW5lIGRvIG5vdCBzb3J0IGludGVydmFsc1xuICAgLy8gbGVzcyB0aGFuIDEwXG4gICB0aGlzLmluc2VydGlvblNvcnRTdGF0aWMoKTtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBpcyBhIHNwZWNpYWwgc29ydGVyIHRoYXQgc29ydHMgYW4gaW50ZWdlciBhcnJheS5cbiogVGhlIG1ldGhvZCBpcyBzcGVjaWFsIGJlY2F1c2UgdGhlIGVsZW1lbnRzIGEgYW5kIGIgb2YgdGhlIGFycmF5XG4qIGFyZSBub3QgY29tcGFyZWQgYXMgaW50ZWdlciB2YWx1ZXMuIFRoZSBlbGVtZW50cyBvZiBvdGhlciBhcnJheVxuKiAoYXJyYXkgb2Yga2V5cykgd2l0aCBpbmRpY2VzIGEgYW5kIGIgYXJlIGNvbXBhcmVkIGluc3RlYWQuIElmIHR3b1xuKiBlbGVtZW50cyBoYXZlIGVxdWFsIGtleXMsIHRoZW4gdGhlIHJlbGF0aXZlIG9yZGVyaW5nIG9mIHRoZXNlXG4qIGVsZW1lbnRzIGlzIHByZXNlcnZlZC5cbiogIFBhcmFtZXRoZXJzOlxuKiAgICAgIG4gLSBpcyB0aGUgbnVtYmVyIG9mIGVsZW1lbnRzIGluIHRoZSBpbnRlZ2VyIGFycmF5O1xuKiAgICAgIGluZGV4IC0gdGhpcyBpcyB0aGUgaW50ZWdlciBhcnJheSB0byBiZSBzb3J0ZWQ7XG4qICAgICAga2V5IC0gdGhpcyBhcnJheSBhY3RzIGFzIGtleXMgZm9yIGNvbXBhcmlzb24gb2YgdHdvIGludGVnZXJcbiogICAgICAgICAgICAgIGVsZW1lbnRzLiBJdCBpcyBhc3N1bWVkIHRoYXQgZWxlbWVudHMgb2YgdGhlIGFycmF5XG4qICAgICAgICAgICAgICBpbmRleCBhcmUgaW4gdGhlIHJhbmdlIFswIC4uIGtleS5sZW5ndGgtMV0uXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5pbmRleFNvcnQgPSBmdW5jdGlvbiAobiwgaW5kZXgsIGtleSlcbntcbiAgIC8vIFdlIHdhbnQgdG8gcmVkdWNlIHRoZSBwcm9ibGVtIHRvIHRoZSBzaW1wbGUgc3RhdGljIHNvcnRpbmcuXG4gICAvLyBGaXJzdCBhIG1hcHBpbmcgZiBmcm9tIHRoZSBzZXQgezAsLi4sbi0xfSB0byB0aGUgc2V0IG9mXG4gICAvLyBlbGVtZW50cyBpcyBkZWZpbmVkIGJ5IGYoaSkgLT4gaW5kZXhbaV0uIFRoZW4gdGhlIHByb2JsZW1cbiAgIC8vIHNpbXBseSByZWR1Y2VzIHRvIHRoZSBzdGF0aWMgc29ydCB3aGVyZSBpLXRoIGtleSB2YWx1ZSBpc1xuICAgLy8ga2V5W2YoaSldLlxuXG4gICAvLyBhbGxvY2F0ZSBtZW1vcnkgZm9yIHRoZSB0ZW1wb3JhcnkgYXJyYXlzIGZvciB0aGUgc3RhdGljIHNvcnRcbiAgIHZhciB0ZW1wSW5kZXggPSBbXTtcbiAgIHZhciB0ZW1wS2V5ID0gW107XG5cbiAgIC8vIHRoaXMgYXJyYXkgd2lsbCBzdG9yZSB0aGUgY29weSBvZiBhcnJheSBpbmRleFxuICAgdmFyIG9sZEluZGV4ID0gW107XG5cbiAgIGZvciAodmFyIGkgPSAwOyBpIDwgbjsgaSsrKVxuICAge1xuICAgICAgIC8vIHN0b3JlIHRoZSBrZXkgZm9yIHN0YXRpYyBzb3J0XG4gICAgICAgdGVtcEtleVtpXSA9IGtleVtpbmRleFtpXV07XG5cbiAgICAgICAvLyByZW1lbWJlciB0aGUgaS10aCB2YWx1ZSBvZiB0aGUgaW5kZXggYXJyYXlcbiAgICAgICBvbGRJbmRleFtpXSA9IGluZGV4W2ldO1xuICAgfVxuXG4gICAvLyBzb3J0IHRoZSBhdXhpbGlhcnkgYXJyYXlzXG4gICB0aGlzLnNvcnQobiwgdGVtcEtleSwgdGVtcEluZGV4KTtcblxuICAgLy8gYW5kIHB1dCB0aGUgcmVzdWx0IGJhY2sgaW4gdGhlIGluZGV4IGFycmF5XG5cbiAgIGZvciAodmFyIGkgPSAwOyBpIDwgbjsgaSsrKVxuICAgICAgIGluZGV4W2ldID0gb2xkSW5kZXhbdGVtcEluZGV4W2ldXTtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBpcyBhIG5vbi1zdGF0aWMgc29ydGVyLlxuKiAgUGFyYW1ldGhlcnM6XG4qICAgICAgbiAtIGlzIHRoZSBudW1iZXIgb2YgZG91YmxlcyB0byBiZSBzb3J0ZWQ7XG4qICAgICAga2V5IC0gaXMgYXJyYXkgb2YgZG91YmxlcyB0byBiZSBzb3J0ZWQuXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5zb3J0ID0gZnVuY3Rpb24gKG4sIGtleSlcbntcbiAgIC8vIHN0b3JlIGFsbCBwYXJhbWV0aGVycyBhcyB0aGUgaW5zdGFuY2UgdmFyaWFibGVzXG4gICB0aGlzLm4gPSBuO1xuICAgdGhpcy5rZXkgPSBrZXk7XG5cbiAgIC8vIGNhbGwgdGhlIHJlY3Vyc2l2ZSBzb3J0aW5nIG1ldGhvZCBvbiB0aGUgd2hvbGUgYXJyYXlcbiAgIHRoaXMucmVjU29ydE5vblN0YXRpYygwLCB0aGlzLm4tMSk7XG5cbiAgIC8vIGRvIHRoZSBmaW5hbCBzb3J0IHdpdGggdGhlIGluc2VydGlvbiBzb3J0aW5nIGFsZ29yaXRobVxuICAgLy8gYmVjYXVzZSB0aGUgcmVjdXJzaXZlIHF1aWNrc29ydCByb3V0aW5lIGRvIG5vdCBzb3J0IGludGVydmFsc1xuICAgLy8gbGVzcyB0aGFuIDEwXG4gICB0aGlzLmluc2VydGlvblNvcnROb25TdGF0aWMoKTtcbn07XG5cbi8vLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuLy8gU2VjdGlvbjogTWV0aG9kcyBmb3Igc3RhdGljIHNvcnRlclxuLy8tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG5cbi8qKlxuICogVGhpcyBtZXRob2QgZG8gdGhlIHJlY3Vyc2l2ZSBzb3J0aW5nIHdpdGggdGhlIHF1aWNrc29ydFxuICogYWxnb3JpdGhtIGZvciBzdGF0aWMgc29ydGVyLiBUaGUgaW50ZXJ2YWwgdG8gYmUgc29ydGVkIGlzXG4gKiBzcGVjaWZpZWQgYnkgdGhlIHBhcmFtZXRoZXJzLlxuICovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLnJlY1NvcnRTdGF0aWMgPSBmdW5jdGlvbiAobGVmdCwgcmlnaHQpXG57XG4gICAgLy8gcmV0dXJuIGltbWVkaWF0ZWx5IGlmIHRoZSBpbnRlcnZhbCBzcGVjaWZpZWQgaXMgdG9vIHNob3J0XG4gICAgaWYgKGxlZnQrMTAgPiByaWdodClcbiAgICB7XG4gICAgICAgIHJldHVybjtcbiAgICB9XG5cbiAgICAvLyBjaG9vc2UgdGhlIHBpdm90IGFuZCBzd2FwIGl0IHdpdGggdGhlIGxhc3QgZWxlbWVudCBvZlxuICAgIC8vIHRoZSBnaXZlbiBpbnRlcnZhbFxuICAgIHZhciBwaXZvdCA9IHRoaXMubWVkaWFuM1N0YXRpYyhsZWZ0LCByaWdodCk7XG5cbiAgICAvLyBpdGVyYXRlIHRocm91Z2ggdGhlIGdpdmVuIGludGVydmFsIGZyb20gYm90aCBlbmRzXG4gICAgLy8gc2ltdWx0YW5lb3VzbHlcbiAgICB2YXIgaSA9IGxlZnQ7XG4gICAgdmFyIGogPSByaWdodC0xO1xuXG4gICAgZm9yICg7OylcbiAgICB7XG4gICAgICAgIC8vIGluY3JlbWVudCBpIHdoaWxlIGktdGggZWxlbWVudCBpcyBsZXNzIHRoYW4gcGl2b3RcbiAgICAgICAgZG9cbiAgICAgICAge1xuICAgICAgICAgICAgaSsrO1xuICAgICAgICB9XG4gICAgICAgIHdoaWxlICh0aGlzLmNtcCh0aGlzLmluZGV4W2ldLCBwaXZvdCkgPT09IC0xKTtcblxuICAgICAgICAvLyBkZWNyZW1lbnQgaiB3aGlsZSBqLXRoIGVsZW1lbnQgaXMgZ3JlYXRlciB0aGFuIHBpdm90XG4gICAgICAgIGRvXG4gICAgICAgIHtcbiAgICAgICAgICAgIGotLTtcbiAgICAgICAgfVxuICAgICAgICB3aGlsZSAodGhpcy5jbXAodGhpcy5pbmRleFtqXSwgcGl2b3QpID09PSAxKTtcblxuICAgICAgICAvLyBpZiBpIGFuZCBqIGNyb3NzIHRoZW4gd2UgYXJlIGRvbmUsIG90aGVyd2lzZSBzd2FwIGktdGhcbiAgICAgICAgLy8gYW5kIGotdGggZWxlbWVudHNcblxuICAgICAgICBpZiAoaSA8IGopXG4gICAgICAgIHtcbiAgICAgICAgICAgIHRoaXMuc3dhcFN0YXRpYyhpLGopO1xuICAgICAgICB9XG4gICAgICAgIGVsc2VcbiAgICAgICAge1xuICAgICAgICAgICAgYnJlYWs7XG4gICAgICAgIH1cbiAgICB9XG5cbiAgICAvLyBwbGFjZSB0aGUgcGl2b3QgaW4gdGhlIHJpZ2h0IHBsYWNlXG4gICAgdGhpcy5zd2FwU3RhdGljKGksIHJpZ2h0LTEpO1xuXG4gICAgLy8gcmVjdXJzaXZlbHkgc29ydCBib3RoIHBhcnRzIHRvIHRoZSBsZWZ0IGFuZCB0byB0aGUgcmlnaHRcbiAgICAvLyBmcm9tIHRoZSBwaXZvdFxuICAgIHRoaXMucmVjU29ydFN0YXRpYyhsZWZ0LCBpLTEpO1xuICAgIHRoaXMucmVjU29ydFN0YXRpYyhpKzEsIHJpZ2h0KTtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBjaG9vc2VzIHRoZSBtaWRkbGUgZWxlbWVudCBvZiBsZWZ0LCByaWdodCBhbmQgY2VudGVyXG4qIGVsZW1lbnQgb2YgZ2l2ZW4gaW50ZXJ2YWwgYW5kIHJldHVybnMgdGhlIGluZGV4IG9mIHRoaXMgZWxlbWVudC5cbiogVGhpcyBtZXRob2QgaXMgdXNlZCBieSBzdGF0aWMgc29ydGVyLlxuKi9cblBvbHlvbWlub1F1aWNrU29ydC5wcm90b3R5cGUubWVkaWFuM1N0YXRpYyA9IGZ1bmN0aW9uIChsZWZ0LCByaWdodClcbntcbiAgIC8vIGNhbGN1bGF0ZXMgdGhlIGNlbnRlciBvZiB0aGUgZ2l2ZW4gaW50ZXJ2YWxcbiAgIHZhciBjZW50ZXIgPSAobGVmdCtyaWdodCkvMjtcblxuICAgLy8gaWYgbGVmdCBlbGVtZW50IGlzIGdyZWF0ZXIgdGhhbiBjZW50ZXIgZWxlbWVudCwgc3dhcCB0aGVtXG4gICBpZiAodGhpcy5jbXAodGhpcy5pbmRleFtsZWZ0XSwgdGhpcy5pbmRleFtjZW50ZXJdKSA9PT0gMSlcbiAgIHtcbiAgICAgICB0aGlzLnN3YXBTdGF0aWMobGVmdCxjZW50ZXIpO1xuICAgfVxuICAgXG4gICAvLyBpZiBsZWZ0IGVsZW1lbnQgaXMgZ3JlYXRlciB0aGFuIHJpZ2h0IGVsZW1lbnQsIHN3YXAgdGhlbVxuICAgaWYgKHRoaXMuY21wKHRoaXMuaW5kZXhbbGVmdF0sIHRoaXMuaW5kZXhbcmlnaHRdKSA9PT0gMSlcbiAgIHtcbiAgICAgICB0aGlzLnN3YXBTdGF0aWMobGVmdCxyaWdodCk7XG4gICB9XG4gICBcbiAgIC8vIGlmIGNlbnRlciBlbGVtZW50IGlzIGdyZWF0ZXIgdGhhbiByaWdodCBlbGVtZW50LCBzd2FwIHRoZW1cbiAgIGlmICh0aGlzLmNtcCh0aGlzLmluZGV4W2NlbnRlcl0sIHRoaXMuaW5kZXhbcmlnaHRdKSA9PT0gMSlcbiAgIHtcbiAgICAgICB0aGlzLnN3YXBTdGF0aWMoY2VudGVyLHJpZ2h0KTtcbiAgIH1cbiAgIFxuICAgLy8gbm93IHRoZSBjZW50ZXIgZWxlbWVudCBpcyBsZXNzIG9yIGVxdWFsIHRoYW4gcmlnaHQgZWxlbWVudFxuICAgLy8gYW5kIGdyZWF0ZXIgb3IgZXF1YWwgdGhhbiBsZWZ0IGVsZW1lbnRcblxuICAgLy8gbW92ZSB0aGUgY2VudGVyIGVsZW1lbnQgdG8gdGhlIHJpZ2h0IHNpZGUgYnkgc3dhcGluZyBpdCB3aXRoXG4gICAvLyB0aGUgb25lIGJlZm9yZSB0aGUgcmlnaHQgZWxlbWVudFxuICAgdGhpcy5zd2FwU3RhdGljKGNlbnRlciwgcmlnaHQtMSk7XG5cbiAgIC8vIHJldHVybiB0aGUgcGl2b3QncyBpbmRleFxuICAgcmV0dXJuIHRoaXMuaW5kZXhbcmlnaHQtMV07XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgY29tcGFyZXMgdHdvIGVsZW1lbnRzIHNwZWNpZmllZCB3aXRoIHRoZWlyIGluZGljZXMuXG4qIEl0IHJldHVybnMgLTEsIGlmIGktdGggZWxlbWVudCBpcyBsZXNzIHRoYW4gai10aCBlbGVtZW50LCBhbmQgMSxcbiogaWYgai10aCBlbGVtZW50IGlzIGxlc3MgdGhhbiBpLXRoIGVsZW1lbnQuIElmIGJvdGggZWxlbWVudHMgYXJlXG4qIGVxdWFsIHRoZW4gdGhlaXIgaW5kaWNlcyBpIGFuZCBqIGFyZSBjb21wYXJlZC4gVGhpcyBtZXRob2QgaXNcbiogdXNlZCBvbmx5IGJ5IHN0YXRpYyBzb3J0ZXIuXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5jbXAgPSBmdW5jdGlvbiAoaSwgailcbntcbiAgIGlmICh0aGlzLmtleVtpXSA8IHRoaXMua2V5W2pdKVxuICAge1xuICAgICAgIHJldHVybiAtMTtcbiAgIH1cbiAgIFxuICAgaWYgKHRoaXMua2V5W2ldID4gdGhpcy5rZXlbal0pXG4gICB7XG4gICAgICAgcmV0dXJuIDE7XG4gICB9XG4gICBcbiAgIGlmIChpIDwgailcbiAgIHtcbiAgICAgICByZXR1cm4gLTE7XG4gICB9XG4gICBcbiAgIGlmIChpID4gailcbiAgIHtcbiAgICAgICByZXR1cm4gMTtcbiAgIH1cbiAgIFxuICAgcmV0dXJuIDA7XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2Qgc3dhcHMgdGhlIGVsZW1lbnRzIGJ5IHN3YXBwaW5nIHRoZWlyIGluZGljZXMuIFVzZWRcbiogb25seSBieSBzdGF0aWMgc29ydGVyLlxuKi9cblBvbHlvbWlub1F1aWNrU29ydC5wcm90b3R5cGUuc3dhcFN0YXRpYyA9IGZ1bmN0aW9uIChpLCBqKVxue1xuICAgdmFyIHRlbXAgPSB0aGlzLmluZGV4W2ldO1xuICAgdGhpcy5pbmRleFtpXSA9IHRoaXMuaW5kZXhbal07XG4gICB0aGlzLmluZGV4W2pdID0gdGVtcDtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBzb3J0cyB0aGUgYXJyYXkgd2l0aCB0aGUgaW5zZXJ0aW9uIHNvcnQgYWxnb3JpdGhtLlxuKiBUaGlzIG1ldGhvZCBpcyB1c2VkIG9ubHkgYnkgc3RhdGljIHNvcnRlZCBhbmQgdGhlcmVmb3JlIGl0IHdvcmtzXG4qIHdpdGggYXJyYXkgb2YgaW5kaWNlcyBpbnN0ZWFkIG9mIGlucHV0IGFycmF5IGl0c2VsZi5cbiovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLmluc2VydGlvblNvcnRTdGF0aWMgPSBmdW5jdGlvbiAoKVxue1xuICAgLy8gaXRlcmF0ZSB0aHJvdWdoIGFsbCBlbGVtZW50c1xuICAgZm9yICh2YXIgaSA9IDE7IGkgPCB0aGlzLm47IGkrKylcbiAgIHtcbiAgICAgICAvLyBzdG9yZXMgdGhlIGluZGV4IG9mIGN1cnJlbnQgZWxlbWVudFxuICAgICAgIHZhciB0ZW1wID0gdGhpcy5pbmRleFtpXTtcblxuICAgICAgIC8vIHNlYXJjaCB0aGUgcmlnaHQgc3BvdCBmb3IgY3VycmVudCBlbGVtZW50IGJ5IGl0ZXJhdGluZ1xuICAgICAgIC8vIGJhY2t3YXJkcyBhbmQgcHVzaGluZyBncmVhdGVyIGVsZW1lbnRzIG9uZSBwbGFjZSB1cFxuICAgICAgIHZhciBqID0gaTtcblxuICAgICAgIHdoaWxlIChqID49IDEgJiYgdGhpcy5jbXAodGhpcy5pbmRleFtqLTFdLCB0ZW1wKSA9PT0gMSlcbiAgICAgICB7XG4gICAgICAgICAgIC8vIGotdGggZWxlbWVudCBpcyBncmVhdGVyIHRoYW4gY3VycmVudCBzbyBtb3ZlIGl0IHVwXG4gICAgICAgICAgIHRoaXMuaW5kZXhbal0gPSB0aGlzLmluZGV4W2otMV07XG4gICAgICAgICAgIGotLTtcbiAgICAgICB9XG5cbiAgICAgICAvLyB3ZSBmb3VuZCB0aGUgbmV3IGhvbWUgZm9yIGN1cnJlbnQgZWxlbWVudCBzbyBzdG9yZSBpdFxuICAgICAgIHRoaXMuaW5kZXhbal0gPSB0ZW1wO1xuICAgfVxufTtcblxuLy8tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG4vLyBTZWN0aW9uOiBNZXRob2RzIGZvciBub24tc3RhdGljIHNvcnRlclxuLy8tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tXG5cbi8qKlxuICogVGhpcyBtZXRob2QgZG8gdGhlIHJlY3Vyc2l2ZSBzb3J0aW5nIHdpdGggdGhlIHF1aWNrc29ydFxuICogYWxnb3JpdGhtIGZvciBub24tc3RhdGljIHNvcnRlci4gVGhlIGludGVydmFsIHRvIGJlIHNvcnRlZCBpc1xuICogc3BlY2lmaWVkIGJ5IHRoZSBwYXJhbWV0aGVycy5cbiAqL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5yZWNTb3J0Tm9uU3RhdGljID0gZnVuY3Rpb24gKGxlZnQsIHJpZ2h0KVxue1xuICAgIC8vIHJldHVybiBpbW1lZGlhdGVseSBpZiB0aGUgaW50ZXJ2YWwgc3BlY2lmaWVkIGlzIHRvbyBzaG9ydFxuICAgIGlmIChsZWZ0KzEwID4gcmlnaHQpXG4gICAge1xuICAgICAgICByZXR1cm47XG4gICAgfVxuICAgIFxuICAgIC8vIGNob29zZSB0aGUgcGl2b3QgYW5kIHN3YXAgaXQgd2l0aCB0aGUgbGFzdCBlbGVtZW50IG9mXG4gICAgLy8gdGhlIGdpdmVuIGludGVydmFsXG4gICAgdmFyIHBpdm90ID0gdGhpcy5tZWRpYW4zTm9uU3RhdGljKGxlZnQsIHJpZ2h0KTtcblxuICAgIC8vIGl0ZXJhdGUgdGhyb3VnaCB0aGUgZ2l2ZW4gaW50ZXJ2YWwgZnJvbSBib3RoIGVuZHNcbiAgICAvLyBzaW11bHRhbmVvdXNseVxuICAgIHZhciBpID0gbGVmdDtcbiAgICB2YXIgaiA9IHJpZ2h0LTE7XG5cbiAgICBmb3IgKDs7KVxuICAgIHtcbiAgICAgICAgLy8gaW5jcmVtZW50IGkgd2hpbGUgaS10aCBlbGVtZW50IGlzIGxlc3MgdGhhbiBwaXZvdFxuICAgICAgICBkb1xuICAgICAgICB7XG4gICAgICAgICAgICBpKys7XG4gICAgICAgIH1cbiAgICAgICAgd2hpbGUgKHRoaXMua2V5W2ldIDwgcGl2b3QpO1xuXG4gICAgICAgIC8vIGRlY3JlbWVudCBqIHdoaWxlIGotdGggZWxlbWVudCBpcyBncmVhdGVyIHRoYW4gcGl2b3RcbiAgICAgICAgZG9cbiAgICAgICAge1xuICAgICAgICAgICAgai0tO1xuICAgICAgICB9XG4gICAgICAgIHdoaWxlICh0aGlzLmtleVtqXSA+IHBpdm90KTtcblxuICAgICAgICAvLyBpZiBpIGFuZCBqIGNyb3NzIHRoZW4gd2UgYXJlIGRvbmUsIG90aGVyd2lzZSBzd2FwIGktdGhcbiAgICAgICAgLy8gYW5kIGotdGggZWxlbWVudHNcbiAgICAgICAgaWYgKGkgPCBqKVxuICAgICAgICB7XG4gICAgICAgICAgICB0aGlzLnN3YXBOb25TdGF0aWMoaSwgaik7XG4gICAgICAgIH1cbiAgICAgICAgZWxzZVxuICAgICAgICB7XG4gICAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgIH1cblxuICAgIC8vIHBsYWNlIHRoZSBwaXZvdCBpbiB0aGUgcmlnaHQgcGxhY2VcbiAgICB0aGlzLnN3YXBOb25TdGF0aWMoaSwgcmlnaHQtMSk7XG5cbiAgICAvLyByZWN1cnNpdmVseSBzb3J0IGJvdGggcGFydHMgdG8gdGhlIGxlZnQgYW5kIHRvIHRoZSByaWdodFxuICAgIC8vIGZyb20gdGhlIHBpdm90XG4gICAgdGhpcy5yZWNTb3J0Tm9uU3RhdGljKGxlZnQsIGktMSk7XG4gICAgdGhpcy5yZWNTb3J0Tm9uU3RhdGljKGkrMSwgcmlnaHQpO1xufTtcblxuLyoqXG4qIFRoaXMgbWV0aG9kIHJldHVybnMgdGhlIG1pZGRsZSBlbGVtZW50IG9mIGxlZnQsIHJpZ2h0IGFuZCBjZW50ZXJcbiogZWxlbWVudC4gVGhpcyBtZXRob2QgaXMgdXNlZCBieSBub24tc3RhdGljIHNvcnRlci5cbiovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLm1lZGlhbjNOb25TdGF0aWMgPSBmdW5jdGlvbiAobGVmdCwgcmlnaHQpXG57XG4gICAgLy8gY2FsY3VsYXRlcyB0aGUgY2VudGVyIG9mIHRoZSBnaXZlbiBpbnRlcnZhbFxuICAgIHZhciBjZW50ZXIgPSAobGVmdCtyaWdodCkvMjtcblxuICAgIC8vIGlmIGxlZnQgZWxlbWVudCBpcyBncmVhdGVyIHRoYW4gY2VudGVyIGVsZW1lbnQsIHN3YXAgdGhlbVxuICAgIGlmICh0aGlzLmtleVtsZWZ0XSA+IHRoaXMua2V5W2NlbnRlcl0pXG4gICAge1xuICAgICAgICB0aGlzLnN3YXBOb25TdGF0aWMobGVmdCwgY2VudGVyKTtcbiAgICB9XG4gICBcbiAgICAvLyBpZiBsZWZ0IGVsZW1lbnQgaXMgZ3JlYXRlciB0aGFuIHJpZ2h0IGVsZW1lbnQsIHN3YXAgdGhlbVxuICAgIGlmICh0aGlzLmtleVtsZWZ0XSA+IHRoaXMua2V5W3JpZ2h0XSlcbiAgICB7XG4gICAgICAgIHRoaXMuc3dhcE5vblN0YXRpYyhsZWZ0LCByaWdodCk7XG4gICAgfVxuICAgIFxuICAgIC8vIGlmIGNlbnRlciBlbGVtZW50IGlzIGdyZWF0ZXIgdGhhbiByaWdodCBlbGVtZW50LCBzd2FwIHRoZW1cbiAgICBpZiAodGhpcy5rZXlbY2VudGVyXSA+IHRoaXMua2V5W3JpZ2h0XSlcbiAgICB7XG4gICAgICAgIHRoaXMuc3dhcE5vblN0YXRpYyhjZW50ZXIsIHJpZ2h0KTtcbiAgICB9XG4gICAgXG4gICAgLy8gbm93IHRoZSBjZW50ZXIgZWxlbWVudCBpcyBsZXNzIG9yIGVxdWFsIHRoYW4gcmlnaHQgZWxlbWVudFxuICAgIC8vIGFuZCBncmVhdGVyIG9yIGVxdWFsIHRoYW4gbGVmdCBlbGVtZW50XG5cbiAgICAvLyBtb3ZlIHRoZSBjZW50ZXIgZWxlbWVudCB0byB0aGUgcmlnaHQgc2lkZSBieSBzd2FwaW5nIGl0IHdpdGhcbiAgICAvLyB0aGUgb25lIGJlZm9yZSB0aGUgcmlnaHQgZWxlbWVudFxuICAgdGhpcy5zd2FwTm9uU3RhdGljKGNlbnRlciwgcmlnaHQtMSk7XG5cbiAgIC8vIHJldHVybiB0aGUgcGl2b3RcbiAgIHJldHVybiB0aGlzLmtleVtyaWdodC0xXTtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBzd2FwcyB0aGUgZWxlbWVudHMuIFVzZWQgb25seSBieSBub24tc3RhdGljIHNvcnRlci5cbiovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLnN3YXBOb25TdGF0aWMgPSBmdW5jdGlvbiAoaSwgailcbntcbiAgIHZhciB0ZW1wID0gdGhpcy5rZXlbaV07XG4gICB0aGlzLmtleVtpXSA9IHRoaXMua2V5W2pdO1xuICAgdGhpcy5rZXlbal0gPSB0ZW1wO1xufTtcblxuLyoqXG4qIFRoaXMgbWV0aG9kIHNvcnRzIHRoZSBhcnJheSB3aXRoIHRoZSBpbnNlcnRpb24gc29ydCBhbGdvcml0aG0uXG4qIFRoaXMgbWV0aG9kIGlzIHVzZWQgb25seSBieSBub24tc3RhdGljIHNvcnRlZC5cbiovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLmluc2VydGlvblNvcnROb25TdGF0aWMgPSBmdW5jdGlvbiAoKVxue1xuICAgIC8vIGl0ZXJhdGUgdGhyb3VnaCBhbGwgZWxlbWVudHNcbiAgICBmb3IgKHZhciBpID0gMTsgaSA8IHRoaXMubjsgaSsrKVxuICAgIHtcbiAgICAgICAgLy8gc3RvcmVzIHRoZSBjdXJyZW50IGVsZW1lbnRcbiAgICAgICAgdmFyIHRlbXAgPSB0aGlzLmtleVtpXTtcblxuICAgICAgICAvLyBzZWFyY2ggdGhlIHJpZ2h0IHNwb3QgZm9yIGN1cnJlbnQgZWxlbWVudCBieSBpdGVyYXRpbmdcbiAgICAgICAgLy8gYmFja3dhcmRzIGFuZCBwdXNoaW5nIGdyZWF0ZXIgZWxlbWVudHMgb25lIHBsYWNlIHVwXG4gICAgICAgIHZhciBqID0gaTtcblxuICAgICAgICB3aGlsZSAoaiA+PSAxICYmIHRoaXMua2V5W2otMV0gPiB0ZW1wKVxuICAgICAgICB7XG4gICAgICAgICAgICAvLyBqLTEtdGggZWxlbWVudCBpcyBncmVhdGVyIHRoYW4gY3VycmVudCBzbyBtb3ZlIGl0IHVwXG4gICAgICAgICAgICB0aGlzLmtleVtqXSA9IHRoaXMua2V5W2otMV07XG4gICAgICAgICAgICBqLS07XG4gICAgICAgIH1cbiAgICAgICBcbiAgICAgICAgLy8gd2UgZm91bmQgdGhlIG5ldyBob21lIGZvciBjdXJyZW50IGVsZW1lbnQgc28gc3RvcmUgaXRcbiAgICAgICAgdGhpcy5rZXlbal0gPSB0ZW1wO1xuICAgIH1cbn07XG5cbm1vZHVsZS5leHBvcnRzID0gUG9seW9taW5vUXVpY2tTb3J0OyIsImZ1bmN0aW9uIFJhbmRvbVNlZWQoKSB7XG59XG5SYW5kb21TZWVkLnNlZWQgPSAxO1xuUmFuZG9tU2VlZC54ID0gMDtcblxuUmFuZG9tU2VlZC5uZXh0RG91YmxlID0gZnVuY3Rpb24gKCkge1xuICBSYW5kb21TZWVkLnggPSBNYXRoLnNpbihSYW5kb21TZWVkLnNlZWQrKykgKiAxMDAwMDtcbiAgcmV0dXJuIFJhbmRvbVNlZWQueCAtIE1hdGguZmxvb3IoUmFuZG9tU2VlZC54KTtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gUmFuZG9tU2VlZDtcbiIsInZhciBSYW5kb21TZWVkID0gcmVxdWlyZSgnLi9SYW5kb21TZWVkJyk7XHJcbnZhciBQb2x5b21pbm8gPSByZXF1aXJlKCcuL1BvbHlvbWlubycpO1xyXG52YXIgUG9pbnQgPSByZXF1aXJlKCcuL1BvaW50Jyk7XHJcblxyXG5mdW5jdGlvbiBSZWN0UHJvYygpIHtcclxufVxyXG5cclxuUmVjdFByb2MuQXNwZWN0UmF0aW8gPSAoMS4wIC8gMS4wKTsvLyB5c2l6ZS94c2l6ZVxyXG5cclxuUmVjdFByb2MuUGxhY2VSYW5kb21seSA9IGZ1bmN0aW9uIChyTiwgclgxLCByWTEsIHJMLCBySClcclxue1xyXG4gICAgdmFyIGluZGV4QXJyYXkgPSBbXTtcclxuXHJcbiAgICB2YXIgc3VtTCA9IDA7XHJcbiAgICB2YXIgc3VtSCA9IDA7XHJcblxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCByTjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHN1bUwgKz0gckxbaV07XHJcbiAgICAgICAgc3VtSCArPSBySFtpXTtcclxuICAgICAgICBpbmRleEFycmF5W2ldID0gaTtcclxuICAgIH1cclxuICAgIFxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCByTjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBhID0gUmFuZG9tU2VlZC5uZXh0RG91YmxlKC8qIFRPRE86IHJOICovKTtcclxuICAgICAgICB2YXIgdG1wID0gaW5kZXhBcnJheVtpXTtcclxuICAgICAgICBpbmRleEFycmF5W2ldID0gaW5kZXhBcnJheVthXTtcclxuICAgICAgICBpbmRleEFycmF5W2FdID0gdG1wO1xyXG4gICAgfVxyXG5cclxuICAgIHN1bUwgLz0gck47XHJcbiAgICBzdW1IIC89IHJOO1xyXG4gICAgdmFyIG51bVJvd3MgPSAoaW50KSAoTWF0aC5zcXJ0KHJOKSArIDAuNDk5OSk7XHJcblxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCByTjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHJYMVtpbmRleEFycmF5W2ldXSA9IChpIC8gbnVtUm93cykgKiBzdW1MO1xyXG4gICAgICAgIHJZMVtpbmRleEFycmF5W2ldXSA9IChpICUgbnVtUm93cykgKiBzdW1IO1xyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgcGFja3MgcmVjdGFuZ2xlcyB1c2luZyBwb2x5b21pbm8gcGFja2luZyBhbGdvcml0aG0uXHJcbiogXHJcbiogQHJldHVyblxyXG4qL1xyXG5SZWN0UHJvYy5wYWNrUmVjdGFuZ2xlc01pbm8gID0gZnVuY3Rpb24gKGJ1ZmZlciwgck4sIHJlY3RhbmdsZXMpXHJcbntcclxuICAgIC8vIG1ha2UgdGhlIGludGVybWVkaWF0ZSBkYXRhIHN0cnVjdHVyZVxyXG4gICAgdmFyIHJYMSA9IFtdO1xyXG4gICAgdmFyIHJZMSA9IFtdO1xyXG4gICAgdmFyIHJXID0gW107XHJcbiAgICB2YXIgckggPSBbXTtcclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJOOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgclgxW2ldID0gcmVjdGFuZ2xlc1tpXS5nZXRDZW50ZXJYKCk7XHJcbiAgICAgICAgclkxW2ldID0gcmVjdGFuZ2xlc1tpXS5nZXRDZW50ZXJZKCk7XHJcbiAgICAgICAgcldbaV0gPSByZWN0YW5nbGVzW2ldLmdldFdpZHRoKCk7XHJcbiAgICAgICAgckhbaV0gPSByZWN0YW5nbGVzW2ldLmdldEhlaWdodCgpO1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIHtcclxuICAgICAgICByWDFbaV0gLT0gcldbaV0gLyAyO1xyXG4gICAgICAgIHJZMVtpXSAtPSBySFtpXSAvIDI7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZG8gdGhlIHBhY2tpbmdcclxuICAgIFJlY3RQcm9jLnBhY2tSZWN0YW5nbGVzTWlubyhidWZmZXIsIHJOLCByWDEsIHJXLCByWTEsIHJILCByZWN0YW5nbGVzKTtcclxuXHJcbiAgICAvLyB0cmFuc2ZlciBiYWNrIHRoZSByZXN1bHRzXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJOOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgclgxW2ldICs9IHJXW2ldIC8gMjtcclxuICAgICAgICByWTFbaV0gKz0gckhbaV0gLyAyO1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIHtcclxuICAgICAgICByZWN0YW5nbGVzW2ldLnNldENlbnRlcihyWDFbaV0sIHJZMVtpXSk7XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBwYWNrcyByZWN0YW5nbGVzIHVzaW5nIHBvbHlvbWlubyBwYWNraW5nIGFsZ29yaXRobS5cclxuKi9cclxuXHJcblJlY3RQcm9jLnBhY2tSZWN0YW5nbGVzTWlubyA9IGZ1bmN0aW9uIChidWZmZXIsIHJOLCByWCwgclcsIHJZLCBySCwgcmVjdGFuZ2xlcylcclxue1xyXG4gICAgaWYgKHJOID09IDApXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG5cclxuICAgIHZhciBzdGVwWCA9IDU7XHJcbiAgICB2YXIgc3RlcFkgPSA1O1xyXG5cclxuICAgIC8vIGR5bmFtaWNhbGx5IGNhbGN1bGF0ZSB0aGUgZ3JpZCBzdGVwXHJcbiAgICAvLyBkb3VibGUgYXJlYSA9IDA7XHJcbiAgICAvL1xyXG4gICAgLy8gZm9yIChpbnQgaSA9IDA7IGkgPCByTjsgaSsrKVxyXG4gICAgLy8ge1xyXG4gICAgLy8gLy8gc3RlcFgrPXJMW2ldK2RlbHRhO1xyXG4gICAgLy8gLy8gc3RlcFkrPXJIW2ldK2RlbHRhO1xyXG4gICAgLy8gYXJlYSArPSAocldbaV0gKyBkZWx0YSkgKiAockhbaV0gKyBkZWx0YSk7XHJcbiAgICAvLyB9XHJcbiAgICAvL1xyXG4gICAgLy8gZG91YmxlIHN0ZXBYID0gTWF0aC5zcXJ0KGFyZWEgLyAock4gKiAxNikpO1xyXG4gICAgLy8gLy8gKHN0ZXBYK3N0ZXBZKS8ock4qOCk7XHJcbiAgICAvL1xyXG5cclxuICAgIC8vIGFkanVzdCByZXNwZWN0aW5nIHRoZSBhc3BlY3QgcmF0aW9cclxuXHJcbiAgICB2YXIgZnN0ZXAgPSAyIC8gKDEgKyBSZWN0UHJvYy5Bc3BlY3RSYXRpbyk7XHJcbiAgICBzdGVwWSA9IHN0ZXBYICogUmVjdFByb2MuQXNwZWN0UmF0aW8gKiBmc3RlcDtcclxuICAgIHN0ZXBYICo9IGZzdGVwO1xyXG5cclxuICAgIC8vIG1ha2UgdGhlIHBvbHlvbWlubyByZXByZXNlbnRhdGlvblxyXG4gICAgdmFyIG1pbm9zID0gW107XHJcblxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCByTjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIC8vIHNpemUgb2YgdGhlIHJlY3RhbmdsZSBpbiBncmlkIHVuaXRzXHJcbiAgICAgICAgdmFyIFcgPSBNYXRoLmNlaWwoKHJXW2ldICsgYnVmZmVyKSAvIHN0ZXBYKTtcclxuICAgICAgICB2YXIgSCA9IE1hdGguY2VpbCgockhbaV0gKyBidWZmZXIpIC8gc3RlcFkpO1xyXG5cclxuICAgICAgICBtaW5vc1tpXSA9IG5ldyBQb2x5b21pbm8oKTtcclxuICAgICAgICBtaW5vc1tpXS5jb29yZCA9IFtdO1xyXG5cclxuICAgICAgICAvLyBjcmVhdGUgdGhlIHBvbHlvbWlubyBjZWxsc1xyXG4gICAgICAgIHZhciBjbnQgPSAwO1xyXG4gICAgICAgIGZvciAodmFyIHkgPSAwOyB5IDwgSDsgeSsrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgZm9yICh2YXIgeCA9IDA7IHggPCBXOyB4KyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIG1pbm9zW2ldLmNvb3JkW2NudF0gPSBuZXcgUG9pbnQoKTtcclxuICAgICAgICAgICAgICAgIG1pbm9zW2ldLmNvb3JkW2NudF0ueCA9IHg7XHJcbiAgICAgICAgICAgICAgICBtaW5vc1tpXS5jb29yZFtjbnQrK10ueSA9IHk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgICAgbWlub3NbaV0ubCA9IGNudDtcclxuICAgICAgICBtaW5vc1tpXS5sYWJlbCA9IHJlY3RhbmdsZXNbaV0ubGFiZWw7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZG8gdGhlIHBhY2tpbmdcclxuICAgIHZhciBwYWNrZXIgPSBuZXcgUG9seW9taW5vUGFja2luZygpO1xyXG4gICAgcGFja2VyLnBhY2sobWlub3MsIHJOKTtcclxuXHJcbiAgICAvLyBnZXQgdGhlIHJlc3VsdHNcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIHtcclxuICAgICAgICByWFtpXSA9IG1pbm9zW2ldLnggKiBzdGVwWDtcclxuICAgICAgICByWVtpXSA9IG1pbm9zW2ldLnkgKiBzdGVwWTtcclxuICAgIH1cclxufTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gUmVjdFByb2M7IiwiZnVuY3Rpb24gUmVjdGFuZ2xlRCh4LCB5LCB3aWR0aCwgaGVpZ2h0KSB7XG4gIHRoaXMueCA9IDA7XG4gIHRoaXMueSA9IDA7XG4gIHRoaXMud2lkdGggPSAwO1xuICB0aGlzLmhlaWdodCA9IDA7XG5cbiAgaWYgKHggIT0gbnVsbCAmJiB5ICE9IG51bGwgJiYgd2lkdGggIT0gbnVsbCAmJiBoZWlnaHQgIT0gbnVsbCkge1xuICAgIHRoaXMueCA9IHg7XG4gICAgdGhpcy55ID0geTtcbiAgICB0aGlzLndpZHRoID0gd2lkdGg7XG4gICAgdGhpcy5oZWlnaHQgPSBoZWlnaHQ7XG4gIH1cbn1cblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0WCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLng7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5zZXRYID0gZnVuY3Rpb24gKHgpXG57XG4gIHRoaXMueCA9IHg7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRZID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMueTtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLnNldFkgPSBmdW5jdGlvbiAoeSlcbntcbiAgdGhpcy55ID0geTtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmdldFdpZHRoID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMud2lkdGg7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5zZXRXaWR0aCA9IGZ1bmN0aW9uICh3aWR0aClcbntcbiAgdGhpcy53aWR0aCA9IHdpZHRoO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0SGVpZ2h0ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuaGVpZ2h0O1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuc2V0SGVpZ2h0ID0gZnVuY3Rpb24gKGhlaWdodClcbntcbiAgdGhpcy5oZWlnaHQgPSBoZWlnaHQ7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRSaWdodCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnggKyB0aGlzLndpZHRoO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0Qm90dG9tID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMueSArIHRoaXMuaGVpZ2h0O1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuaW50ZXJzZWN0cyA9IGZ1bmN0aW9uIChhKVxue1xuICBpZiAodGhpcy5nZXRSaWdodCgpIDwgYS54KVxuICB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgaWYgKHRoaXMuZ2V0Qm90dG9tKCkgPCBhLnkpXG4gIHtcbiAgICByZXR1cm4gZmFsc2U7XG4gIH1cblxuICBpZiAoYS5nZXRSaWdodCgpIDwgdGhpcy54KVxuICB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgaWYgKGEuZ2V0Qm90dG9tKCkgPCB0aGlzLnkpXG4gIHtcbiAgICByZXR1cm4gZmFsc2U7XG4gIH1cblxuICByZXR1cm4gdHJ1ZTtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmdldENlbnRlclggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy54ICsgdGhpcy53aWR0aCAvIDI7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRNaW5YID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuZ2V0WCgpO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0TWF4WCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmdldFgoKSArIHRoaXMud2lkdGg7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRDZW50ZXJZID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMueSArIHRoaXMuaGVpZ2h0IC8gMjtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmdldE1pblkgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5nZXRZKCk7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRNYXhZID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuZ2V0WSgpICsgdGhpcy5oZWlnaHQ7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRXaWR0aEhhbGYgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy53aWR0aCAvIDI7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRIZWlnaHRIYWxmID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuaGVpZ2h0IC8gMjtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gUmVjdGFuZ2xlRDtcbiIsInZhciBDb1NFQ29uc3RhbnRzID0gcmVxdWlyZSgnLi9Db1NFQ29uc3RhbnRzJyk7XG5cbmZ1bmN0aW9uIFNiZ25QRENvbnN0YW50cygpIHtcbn1cblxuLy9TYmduUERDb25zdGFudHMgaW5oZXJpdHMgc3RhdGljIHByb3BzIGluIENvU0VDb25zdGFudHMgXG5mb3IgKHZhciBwcm9wIGluIENvU0VDb25zdGFudHMpIHtcbiAgU2JnblBEQ29uc3RhbnRzW3Byb3BdID0gQ29TRUNvbnN0YW50c1twcm9wXTtcbn1cblxuLy8gQmVsb3cgYXJlIHRoZSBTQkdOIGdseXBoIHNwZWNpZmljIHR5cGVzLlxuU2JnblBEQ29uc3RhbnRzLk1BQ1JPTU9MRUNVTEUgPSBcIm1hY3JvbW9sZWN1bGVcIjtcblNiZ25QRENvbnN0YW50cy5VTklUX09GX0lORk9STUFUSU9OID0gXCJ1bml0IG9mIGluZm9ybWF0aW9uXCI7XG5TYmduUERDb25zdGFudHMuU1RBVEVfVkFSSUFCTEUgPSBcInN0YXRlIHZhcmlhYmxlXCI7XG5TYmduUERDb25zdGFudHMuU09VUkNFX0FORF9TSU5LID0gXCJzb3VyY2UgYW5kIHNpbmtcIjtcblNiZ25QRENvbnN0YW50cy5BU1NPQ0lBVElPTiA9IFwiYXNzb2NpYXRpb25cIjtcblNiZ25QRENvbnN0YW50cy5ESVNTT0NJQVRJT04gPSBcImRpc3NvY2lhdGlvblwiO1xuU2JnblBEQ29uc3RhbnRzLk9NSVRURURfUFJPQ0VTUyA9IFwib21pdHRlZCBwcm9jZXNzXCI7XG5TYmduUERDb25zdGFudHMuVU5DRVJUQUlOX1BST0NFU1MgPSBcInVuY2VydGFpbiBwcm9jZXNzXCI7XG5TYmduUERDb25zdGFudHMuU0lNUExFX0NIRU1JQ0FMID0gXCJzaW1wbGUgY2hlbWljYWxcIjtcblNiZ25QRENvbnN0YW50cy5QUk9DRVNTID0gXCJwcm9jZXNzXCI7XG5TYmduUERDb25zdGFudHMuQ09NUExFWCA9IFwiY29tcGxleFwiO1xuU2JnblBEQ29uc3RhbnRzLkFORCA9IFwiYW5kXCI7XG5TYmduUERDb25zdGFudHMuT1IgPSBcIm9yXCI7XG5TYmduUERDb25zdGFudHMuTk9UID0gXCJub3RcIjtcblNiZ25QRENvbnN0YW50cy5QSEVOT1RZUEUgPSBcInBoZW5vdHlwZVwiO1xuU2JnblBEQ29uc3RhbnRzLlBFUlRVUkJJTkdfQUdFTlQgPSBcInBlcnR1cmJpbmcgYWdlbnRcIjtcblNiZ25QRENvbnN0YW50cy5UQUcgPSBcInRhZ1wiO1xuU2JnblBEQ29uc3RhbnRzLk5VQ0xFSUNfQUNJRF9GRUFUVVJFID0gXCJudWNsZWljIGFjaWQgZmVhdHVyZVwiO1xuU2JnblBEQ29uc3RhbnRzLlVOU1BFQ0lGSUVEX0VOVElUWSA9IFwidW5zcGVjaWZpZWQgZW50aXR5XCI7XG5TYmduUERDb25zdGFudHMuSU5QVVRfUE9SVCA9IFwiaW5wdXRfcG9ydFwiO1xuU2JnblBEQ29uc3RhbnRzLk9VVFBVVF9QT1JUID0gXCJvdXRwdXRfcG9ydFwiO1xuXG4vKipcbiAqICBUaGlzIGNvbXBvdW5kIHR5cGUgaXMgb25seSB1c2VkIHRvIGVuY2xvc2UgYSBwcm9jZXNzIG5vZGUgYW5kIGl0cyB0d28gYXNzb2NpYXRlZCBwb3J0IG5vZGVzIFxuICovXG5TYmduUERDb25zdGFudHMuRFVNTVlfQ09NUE9VTkQgPSBcImR1bW15IGNvbXBvdW5kXCI7XG5cbi8vIEJlbG93IGFyZSB0aGUgU0JHTiBBcmMgc3BlY2lmaWMgdHlwZXMuXG5TYmduUERDb25zdGFudHMuUFJPRFVDVElPTiA9IFwicHJvZHVjdGlvblwiO1xuU2JnblBEQ29uc3RhbnRzLkNPTlNVTVBUSU9OID0gXCJjb25zdW1wdGlvblwiO1xuU2JnblBEQ29uc3RhbnRzLklOSElCSVRJT04gPSBcImluaGliaXRpb25cIjtcblNiZ25QRENvbnN0YW50cy5DQVRBTFlTSVMgPSBcImNhdGFseXNpc1wiO1xuU2JnblBEQ29uc3RhbnRzLk1PRFVMQVRJT04gPSBcIm1vZHVsYXRpb25cIjtcblNiZ25QRENvbnN0YW50cy5TVElNVUxBVElPTiA9IFwic3RpbXVsYXRpb25cIjtcblNiZ25QRENvbnN0YW50cy5ORUNFU1NBUllfU1RJTVVMQVRJT04gPSBcIm5lY2Vzc2FyeSBzdGltdWxhdGlvblwiO1xuXG5TYmduUERDb25zdGFudHMuUklHSURfRURHRV9MRU5HVEggPSAxMDtcblNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFID0gXCJyaWdpZCBlZGdlXCI7XG5cblNiZ25QRENvbnN0YW50cy5QT1JUX05PREVfREVGQVVMVF9XSURUSCA9IDM7XG5TYmduUERDb25zdGFudHMuUE9SVF9OT0RFX0RFRkFVTFRfSEVJR0hUID0gMztcblxuU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX0hPUklaT05UQUxfQlVGRkVSID0gNTtcblNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVIgPSA1O1xuU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTiA9IDIwO1xuU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUlOX1dJRFRIID0gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTiAqIDI7XHRcblxuU2JnblBEQ29uc3RhbnRzLlBIQVNFMV9NQVhfSVRFUkFUSU9OX0NPVU5UID0gMjAwO1xuU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0UgPSAxMDtcblNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX1BFUklPRCA9IDMwO1xuU2JnblBEQ29uc3RhbnRzLlBIQVNFMl9JTklUSUFMX0NPT0xJTkdGQUNUT1IgPSAwLjM7XG5cblNiZ25QRENvbnN0YW50cy5FRkZFQ1RPUl9BTkdMRV9UT0xFUkFOQ0UgPSA0NTtcblNiZ25QRENvbnN0YW50cy5BTkdMRV9UT0xFUkFOQ0UgPSAxMDA7XG5TYmduUERDb25zdGFudHMuUk9UQVRJT05fOTBfREVHUkVFID0gNjA7XG5TYmduUERDb25zdGFudHMuUk9UQVRJT05fMTgwX0RFR1JFRSA9IDAuNTtcblNiZ25QRENvbnN0YW50cy5ST1RBVElPTkFMX0ZPUkNFX0lURVJBVElPTl9DT1VOVCA9IDI7XG5TYmduUERDb25zdGFudHMuUk9UQVRJT05BTF9GT1JDRV9DT05WRVJHRU5DRSA9IDEuMDtcblxubW9kdWxlLmV4cG9ydHMgPSBTYmduUERDb25zdGFudHM7IiwidmFyIENvU0VFZGdlID0gcmVxdWlyZSgnLi9Db1NFRWRnZScpO1xudmFyIFNiZ25QRENvbnN0YW50cyA9IHJlcXVpcmUoJy4vU2JnblBEQ29uc3RhbnRzJyk7XG5cbmZ1bmN0aW9uIFNiZ25QREVkZ2Uoc291cmNlLCB0YXJnZXQsIHZFZGdlKSBcbntcbiAgICBDb1NFRWRnZS5jYWxsKHRoaXMsIHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSk7XG4gICAgXG4gICAgdGhpcy5jb3JyZXNwb25kaW5nQW5nbGUgPSAwO1xuICAgIHRoaXMuaXNQcm9wZXJseU9yaWVudGVkID0gZmFsc2U7XG59XG5cbmZ1bmN0aW9uIFNiZ25QREVkZ2Uoc291cmNlLCB0YXJnZXQsIHZFZGdlLCB0eXBlKSBcbntcbiAgICBDb1NFRWRnZS5jYWxsKHRoaXMsIHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSk7XG4gICAgXG4gICAgdGhpcy50eXBlID0gdHlwZTsgICAgICAgICAgICAgICAgLy8gU3RyaW5nIChmcm9tIExHcmFwaE9iamVjdClcbiAgICB0aGlzLmNvcnJlc3BvbmRpbmdBbmdsZSA9IDA7ICAgICAvLyBpbnRcbiAgICB0aGlzLmlzUHJvcGVybHlPcmllbnRlZCA9IGZhbHNlOyAvLyBib29sZWFuXG59XG5cbkNvU0VFZGdlLnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoQ29TRUVkZ2UucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gQ29TRUVkZ2UpIHtcbiAgU2JnblBERWRnZVtwcm9wXSA9IENvU0VFZGdlW3Byb3BdO1xufVxuXG5TYmduUERFZGdlLnByb3RvdHlwZS5jb3B5ID0gZnVuY3Rpb24gKC8qU2JnblBERWRnZSovIGVkZ2UpIFxue1xuICAgIC8vIFRPRE86IERvIHdlIHJlYWxseSBoYXZlIGFjY2VzcyB0byB0aGVzZSB0d28gZnVuY3Rpb25zP1xuICAgIHRoaXMuc2V0U291cmNlKGVkZ2UuZ2V0U291cmNlKCkpO1xuICAgIHRoaXMuc2V0VGFyZ2V0KGVkZ2UuZ2V0VGFyZ2V0KCkpO1xuICAgIFxuICAgIHRoaXMubGFiZWwgPSBlZGdlLmxhYmVsO1xuICAgIHRoaXMudHlwZSA9IGVkZ2UudHlwZTtcbiAgICB0aGlzLmNvcnJlc3BvbmRpbmdBbmdsZSA9IGVkZ2UuY29ycmVzcG9uZGluZ0FuZ2xlO1xuICAgIHRoaXMuaXNQcm9wZXJseU9yaWVudGVkID0gZWRnZS5pc1Byb3Blcmx5T3JpZW50ZWQ7XG4gICAgdGhpcy5pZGVhbExlbmd0aCA9IGVkZ2UuaWRlYWxMZW5ndGg7XG4gICAgdGhpcy5pc0ludGVyR3JhcGggPSBlZGdlLmlzSW50ZXJHcmFwaDtcbiAgICB0aGlzLmJlbmRwb2ludHMgPSBlZGdlLmJlbmRwb2ludHM7XG4gICAgdGhpcy5pc092ZXJsYXBpbmdTb3VyY2VBbmRUYXJnZXQgPSBlZGdlLmlzT3ZlcmxhcGluZ1NvdXJjZUFuZFRhcmdldDtcbiAgICB0aGlzLmxjYSA9IGVkZ2UubGNhO1xuICAgIHRoaXMubGVuZ3RoID0gZWRnZS5sZW5ndGg7XG4gICAgdGhpcy5sZW5ndGhYID0gZWRnZS5sZW5ndGhYO1xuICAgIHRoaXMubGVuZ3RoWSA9IGVkZ2UubGVuZ3RoWTtcbiAgICB0aGlzLnNvdXJjZUluTGNhID0gZWRnZS5zb3VyY2VJbkxjYTtcbn07XG5cblNiZ25QREVkZ2UucHJvdG90eXBlLmlzRWZmZWN0b3IgPSBmdW5jdGlvbiAoKSBcbntcbiAgICBpZih0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuTU9EVUxBVElPTikgPT09IDAgfHwgXG4gICAgICAgdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLlNUSU1VTEFUSU9OKSA9PT0gMCB8fCBcbiAgICAgICB0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuQ0FUQUxZU0lTKSA9PT0gMCB8fCBcbiAgICAgICB0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuSU5ISUJJVElPTikgPT09IDAgfHwgXG4gICAgICAgdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLk5FQ0VTU0FSWV9TVElNVUxBVElPTikgPT09IDApXG4gICB7XG4gICAgICAgcmV0dXJuIHRydWU7XG4gICB9XG4gICBlbHNlXG4gICB7XG4gICAgICAgcmV0dXJuIGZhbHNlO1xuICAgfVxufTtcblxuU2JnblBERWRnZS5wcm90b3R5cGUuaXNSaWdpZEVkZ2UgPSBmdW5jdGlvbiAoKSBcbntcbiAgICBpZih0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuUklHSURfRURHRSkgPT09IDAgKVxuICAgIHtcbiAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IFNiZ25QREVkZ2U7XG4iLCJ2YXIgSW50ZWdlciA9IHJlcXVpcmUoJy4vSW50ZWdlcicpO1xyXG52YXIgSUdlb21ldHJ5ID0gcmVxdWlyZSgnLi9JR2VvbWV0cnknKTtcclxudmFyIFBvaW50RCA9IHJlcXVpcmUoJy4vUG9pbnREJyk7XHJcbnZhciBSZWN0YW5nbGVEID0gcmVxdWlyZSgnLi9SZWN0YW5nbGVEJyk7XHJcblxyXG52YXIgSGFzaE1hcCA9IHJlcXVpcmUoJy4vSGFzaE1hcCcpO1xyXG52YXIgSGFzaFNldCA9IHJlcXVpcmUoJy4vSGFzaFNldCcpO1xyXG5cclxudmFyIENvU0VMYXlvdXQgPSByZXF1aXJlKCcuL0NvU0VMYXlvdXQnKTtcclxudmFyIFNiZ25QRE5vZGUgPSByZXF1aXJlKCcuL1NiZ25QRE5vZGUnKTtcclxudmFyIFNiZ25QREVkZ2UgPSByZXF1aXJlKCcuL1NiZ25QREVkZ2UnKTtcclxudmFyIFNiZ25Qcm9jZXNzTm9kZSA9IHJlcXVpcmUoJy4vU2JnblByb2Nlc3NOb2RlJyk7XHJcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xyXG5cclxudmFyIE1lbWJlclBhY2sgPSByZXF1aXJlKCcuL01lbWJlclBhY2snKTtcclxudmFyIFJlY3RQcm9jID0gcmVxdWlyZSgnLi9SZWN0UHJvYycpO1xyXG52YXIgQ29tcGFjdGlvbiA9IHJlcXVpcmUoJy4vQ29tcGFjdGlvbicpO1xyXG5cclxuU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bSA9IFxyXG57XHJcbiAgICBUSUxJTkcgOiAwLCBcclxuICAgIFBPTFlPTUlOT19QQUNLSU5HIDogMVxyXG59O1xyXG5cclxuU2JnblBETGF5b3V0Lk9yaWVudGF0aW9uRW51bSA9IFxyXG57XHJcbiAgICBMRUZUX1RPX1JJR0hUIDogMCwgXHJcbiAgICBSSUdIVF9UT19MRUZUIDogMSxcclxuICAgIFRPUF9UT19CT1RUT00gOiAyLCBcclxuICAgIEJPVFRPTV9UT19UT1AgOiAzXHJcbn07XHJcblxyXG5mdW5jdGlvbiBTYmduUERMYXlvdXQoKSBcclxue1xyXG4gICAgQ29TRUxheW91dC5jYWxsKHRoaXMpO1xyXG5cclxuICAgIHRoaXMucm90YXRpb25SYW5kb21pemF0aW9uTWV0aG9kID0gMTtcclxuXHJcbiAgICB0aGlzLmVuaGFuY2VkUmF0aW8gPSAwO1xyXG4gICAgdGhpcy50b3RhbEVmZkNvdW50ID0gMDtcclxuICAgIHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9IFNiZ25QRExheW91dC5EZWZhdWx0Q29tcGFjdGlvbkFsZ29yaXRobUVudW0uVElMSU5HO1xyXG5cclxuICAgIHRoaXMuY2hpbGRHcmFwaE1hcCA9IG5ldyBIYXNoTWFwKCk7ICAgICAgICAgIC8qTWFwPFNiZ25QRE5vZGUsIExHcmFwaD4qL1xyXG4gICAgdGhpcy5jb21wbGV4T3JkZXIgPSBbXTsgICAgICAgICAgICAgICAgICAgICAgLypMaXN0PFNiZ25QRE5vZGU+Ki9cclxuICAgIHRoaXMuZHVtbXlDb21wbGV4TGlzdCA9IFtdOyAgICAgICAgICAgICAgICAgIC8qTGlzdDxTYmduUEROb2RlPiovXHJcbiAgICB0aGlzLmVtcHRpZWREdW1teUNvbXBsZXhNYXAgPSBuZXcgSGFzaE1hcCgpOyAvKk1hcDxTYmduUEROb2RlLCBMR3JhcGg+Ki9cclxuICAgIHRoaXMucHJvY2Vzc05vZGVMaXN0ID0gW107ICAgICAgICAgICAgICAgICAgIC8qTGlzdDxTYmduUHJvY2Vzc05vZGU+Ki9cclxuICAgIFxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5USUxJTkcpXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5tZW1iZXJQYWNrTWFwID0gbmV3IEhhc2hNYXAoKTsgICAgICAvKk1hcDxTYmduUEROb2RlLCBNZW1iZXJQYWNrPiovXHJcbiAgICB9XHJcbn07XHJcblxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShDb1NFTGF5b3V0LnByb3RvdHlwZSk7XHJcblxyXG5mb3IgKHZhciBwcm9wIGluIENvU0VMYXlvdXQpIHtcclxuICBTYmduUERMYXlvdXRbcHJvcF0gPSBDb1NFTGF5b3V0W3Byb3BdO1xyXG59XHJcblxyXG4vKipcclxuICogQE92ZXJyaWRlIFRoaXMgbWV0aG9kIHBlcmZvcm1zIHRoZSBhY3R1YWwgbGF5b3V0IG9uIHRoZSBsLWxldmVsIGNvbXBvdW5kXHJcbiAqICAgICAgICAgICBncmFwaC4gQW4gdXBkYXRlKCkgbmVlZHMgdG8gYmUgY2FsbGVkIGZvciBjaGFuZ2VzIHRvIGJlXHJcbiAqICAgICAgICAgICBwcm9wYWdhdGVkIHRvIHRoZSB2LWxldmVsIGNvbXBvdW5kIGdyYXBoLlxyXG4gKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5ydW5TcHJpbmdFbWJlZGRlciA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIGNvbnNvbGUubG9nKFwiU0JHTi1QRCBMYXlvdXQgaXMgcnVubmluZy4uLlwiKTtcclxuICAgIHRoaXMucGhhc2VOdW1iZXIgPSAxO1xyXG4gICAgdGhpcy5kb1BoYXNlMSgpO1xyXG5cclxuICAgIHRoaXMucGhhc2VOdW1iZXIgPSAyO1xyXG4gICAgdGhpcy5kb1BoYXNlMigpO1xyXG5cclxuICAgIC8vIHVzZWQgdG8gY2FsY3VsYXRlIC0gdG8gbWFrZSBzdXJlXHJcbiAgICB0aGlzLnJlY2FsY1Byb3Blcmx5T3JpZW50ZWRFZGdlcyh0cnVlKTtcclxuXHJcbiAgICBjb25zb2xlLmxvZyhcInN1Y2Nlc3MgcmF0aW86IFwiICsgdGhpcy5zdWNjZXNzUmF0aW8pO1xyXG5cclxuICAgIHRoaXMuZmluYWxFbmhhbmNlbWVudCgpO1xyXG5cclxuICAgIGNvbnNvbGUubG9nKFwiZW5oYW5jZWQgcmF0aW86IFwiICsgdGhpcy5lbmhhbmNlZFJhdGlvKTtcclxuXHJcbiAgICB0aGlzLnJlbW92ZUR1bW15Q29tcG91bmRzKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBBdCB0aGlzIHBoYXNlLCBDb1NFIGlzIGFwcGxpZWQgZm9yIGEgbnVtYmVyIG9mIGl0ZXJhdGlvbnMuXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuZG9QaGFzZTEgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB0aGlzLm1heEl0ZXJhdGlvbnMgPSBTYmduUERDb25zdGFudHMuUEhBU0UxX01BWF9JVEVSQVRJT05fQ09VTlQ7XHJcbiAgICB0aGlzLnRvdGFsSXRlcmF0aW9ucyA9IDA7XHJcblxyXG4gICAgZG9cclxuICAgIHtcclxuICAgICAgICB0aGlzLnRvdGFsSXRlcmF0aW9ucysrO1xyXG4gICAgICAgIGlmICgodGhpcy50b3RhbEl0ZXJhdGlvbnMgJSBTYmduUERDb25zdGFudHMuQ09OVkVSR0VOQ0VfQ0hFQ0tfUEVSSU9EKSA9PT0gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0aGlzLmlzQ29udmVyZ2VkKCkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICB0aGlzLmNvb2xpbmdGYWN0b3IgPSBcclxuICAgICAgICAgICAgICAgIHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3IgKiBcclxuICAgICAgICAgICAgICAgICgodGhpcy5tYXhJdGVyYXRpb25zIC0gdGhpcy50b3RhbEl0ZXJhdGlvbnMpIC8gdGhpcy5tYXhJdGVyYXRpb25zKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHRoaXMudG90YWxEaXNwbGFjZW1lbnQgPSAwO1xyXG5cclxuICAgICAgICB0aGlzLmdyYXBoTWFuYWdlci51cGRhdGVCb3VuZHMoKTtcclxuICAgICAgICB0aGlzLmNhbGNTcHJpbmdGb3JjZXMoKTtcclxuICAgICAgICB0aGlzLmNhbGNSZXB1bHNpb25Gb3JjZXMoKTtcclxuICAgICAgICB0aGlzLmNhbGNHcmF2aXRhdGlvbmFsRm9yY2VzKCk7XHJcbiAgICAgICAgdGhpcy5tb3ZlTm9kZXMoKTtcclxuXHJcbiAgICAgICAgdGhpcy5hbmltYXRlKCk7XHJcbiAgICB9XHJcbiAgICB3aGlsZSAodGhpcy50b3RhbEl0ZXJhdGlvbnMgPCB0aGlzLm1heEl0ZXJhdGlvbnMpO1xyXG5cclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnVwZGF0ZUJvdW5kcygpO1xyXG4gICAgdGhpcy5waGFzZTFJdGVyYXRpb25Db3VudCA9IHRoaXMudG90YWxJdGVyYXRpb25zO1xyXG59O1xyXG5cclxuLyoqXHJcbiogQXQgdGhpcyBwaGFzZSwgbG9jYXRpb24gb2Ygc2luZ2xlIG5vZGVzIGFyZSBhcHByb3hpbWF0ZWQgb2NjYXNpb25hbGx5LlxyXG4qIFJvdGF0aW9uYWwgZm9yY2VzIGFyZSBhcHBsaWVkLiBDb29saW5nIGZhY3RvciBzdGFydHMgZnJvbSBhIHNtYWxsIHZhbHVlXHJcbiogdG8gcHJldmVudCBodWdlIGNoYW5nZXMuXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuZG9QaGFzZTIgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICAvLyBkeW5hbWljIG1heCBpdGVyYXRpb25cclxuICAgIHRoaXMubWF4SXRlcmF0aW9ucyA9IFxyXG4gICAgICAgICAgICBNYXRoLmxvZyh0aGlzLmdldEFsbEVkZ2VzKCkubGVuZ3RoICsgdGhpcy5nZXRBbGxOb2RlcygpLmxlbmd0aCkgKiA0MDA7XHJcblxyXG4gICAgLy8gY29vbGluZyBmYWMgaXMgc21hbGxcclxuICAgIHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3IgPSBTYmduUERDb25zdGFudHMuUEhBU0UyX0lOSVRJQUxfQ09PTElOR0ZBQ1RPUjtcclxuICAgIHRoaXMuY29vbGluZ0ZhY3RvciA9IHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3I7XHJcblxyXG4gICAgdGhpcy50b3RhbEl0ZXJhdGlvbnMgPSAwO1xyXG5cclxuICAgIGRvXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy50b3RhbEl0ZXJhdGlvbnMrKztcclxuXHJcbiAgICAgICAgaWYgKCh0aGlzLnRvdGFsSXRlcmF0aW9ucyAlIFNiZ25QRENvbnN0YW50cy5DT05WRVJHRU5DRV9DSEVDS19QRVJJT0QpID09PSAwKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMuc3VjY2Vzc1JhdGlvID0gdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50IC8gdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZDtcclxuXHJcbiAgICAgICAgICAgICAgICBpZiAodGhpcy5pc0NvbnZlcmdlZCgpICYmIFxyXG4gICAgICAgICAgICAgICAgICAgIHN1Y2Nlc3NSYXRpbyA+PSBTYmduUERDb25zdGFudHMuUk9UQVRJT05BTF9GT1JDRV9DT05WRVJHRU5DRSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBicmVhaztcclxuICAgICAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgICAgICB0aGlzLmNvb2xpbmdGYWN0b3IgPSBcclxuICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy5pbml0aWFsQ29vbGluZ0ZhY3RvciAqIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAoKHRoaXMubWF4SXRlcmF0aW9ucyAtIHRoaXMudG90YWxJdGVyYXRpb25zKSAvIHRoaXMubWF4SXRlcmF0aW9ucyk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICB0aGlzLnRvdGFsRGlzcGxhY2VtZW50ID0gMDtcclxuXHJcbiAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIudXBkYXRlQm91bmRzKCk7XHJcblxyXG4gICAgICAgIHRoaXMuY2FsY1NwcmluZ0ZvcmNlcygpO1xyXG4gICAgICAgIHRoaXMuY2FsY1JlcHVsc2lvbkZvcmNlcygpO1xyXG4gICAgICAgIHRoaXMuY2FsY0dyYXZpdGF0aW9uYWxGb3JjZXMoKTtcclxuICAgICAgICB0aGlzLm1vdmVOb2RlcygpO1xyXG4gICAgICAgIHRoaXMuYW5pbWF0ZSgpO1xyXG4gICAgfVxyXG4gICAgd2hpbGUgKHRoaXMudG90YWxJdGVyYXRpb25zIDwgdGhpcy5tYXhJdGVyYXRpb25zXHJcbiAgICAgICAgICAgICAgICAgICAgJiYgdGhpcy50b3RhbEl0ZXJhdGlvbnMgPCAxMDAwMCk7XHJcblxyXG4gICAgdGhpcy5waGFzZTJJdGVyYXRpb25Db3VudCA9IHRoaXMudG90YWxJdGVyYXRpb25zO1xyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIudXBkYXRlQm91bmRzKCk7XHJcbn07XHJcblxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLm1vdmVOb2RlcyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHRoaXMucHJvcGVybHlPcmllbnRlZEVkZ2VDb3VudCA9IDA7XHJcbiAgICB0aGlzLnRvdGFsRWRnZUNvdW50VG9CZU9yaWVudGVkID0gMDtcclxuXHJcbiAgICAvLyBvbmx5IGNoYW5nZSBzaW5nbGUgbm9kZSBwb3NpdGlvbnMgb24gZWFybHkgc3RhZ2VzXHJcbiAgICBpZiAodGhpcy5oYXNBcHByb3hpbWF0aW9uUGVyaW9kUmVhY2hlZCgpICYmIHRoaXMuY29vbGluZ0ZhY3RvciA+IDAuMDIpXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIG51bU9mUHJvY2Vzc05vZGVzID0gdGhpcy5wcm9jZXNzTm9kZUxpc3QubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGluZGV4OyBpbmRleDxudW1PZlByb2Nlc3NOb2RlczsgaW5kZXgrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvY2Vzc05vZGVMaXN0W2luZGV4XS5hcHBseUFwcHJveGltYXRpb25zKCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZQcm9jZXNzTm9kZXMgPSB0aGlzLnByb2Nlc3NOb2RlTGlzdC5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpbmRleDsgaW5kZXg8bnVtT2ZQcm9jZXNzTm9kZXM7IGluZGV4KyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHNiZ25Qcm9jZXNzTm9kZSA9IHRoaXMucHJvY2Vzc05vZGVMaXN0W2luZGV4XTtcclxuICAgICAgICBcclxuICAgICAgICAvLyBjYWxjdWxhdGUgcm90YXRpb25hbCBmb3JjZXMgZm9yIHBoYXNlIDIgb25seVxyXG4gICAgICAgIGlmICh0aGlzLnBoYXNlTnVtYmVyID09PSAyKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLmNhbGNSb3RhdGlvbmFsRm9yY2VzKCk7XHJcblxyXG4gICAgICAgICAgICB0aGlzLnByb3Blcmx5T3JpZW50ZWRFZGdlQ291bnQgKz0gc2JnblByb2Nlc3NOb2RlLnByb3BlckVkZ2VDb3VudDtcclxuICAgICAgICAgICAgdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZCArPSBcclxuICAgICAgICAgICAgICAgICAgICAoc2JnblByb2Nlc3NOb2RlLmNvbnN1bXB0aW9uRWRnZXMuc2l6ZSgpICsgXHJcbiAgICAgICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5wcm9kdWN0RWRnZXMuc2l6ZSgpICsgXHJcbiAgICAgICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5lZmZlY3RvckVkZ2VzLnNpemUoKSk7XHJcbiAgICAgICAgICAgIHRoaXMuc3VjY2Vzc1JhdGlvID0gXHJcbiAgICAgICAgICAgICAgICAgICAgdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50IC8gdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZDtcclxuICAgICAgICB9XHJcbiAgICAgICAgXHJcbiAgICAgICAgc2JnblByb2Nlc3NOb2RlLnRyYW5zZmVyRm9yY2VzKCk7XHJcblxyXG4gICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5yZXNldEZvcmNlcygpO1xyXG4gICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5pbnB1dFBvcnQucmVzZXRGb3JjZXMoKTtcclxuICAgICAgICBzYmduUHJvY2Vzc05vZGUub3V0cHV0UG9ydC5yZXNldEZvcmNlcygpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGVhY2ggdGltZSwgcm90YXRlIG9uZSBwcm9jZXNzIHRoYXQgd2FudHMgdG8gcm90YXRlXHJcbiAgICBpZiAoKCh0aGlzLnRvdGFsSXRlcmF0aW9ucyAlIFNiZ25QRENvbnN0YW50cy5ST1RBVElPTkFMX0ZPUkNFX0lURVJBVElPTl9DT1VOVCkgPT09IDApICYmIFxyXG4gICAgICAgIHRoaXMucGhhc2VOdW1iZXIgPT09IDIpXHJcbiAgICB7XHJcbiAgICAgICAgcm90YXRlQVByb2Nlc3MoKTtcclxuICAgIH1cclxuICAgICAgICAgICAgXHJcbiAgICAvLyBUT0RPOiBPcmlnaW5hbDogc3VwZXIubW92ZU5vZGVzKCk7XHJcbiAgICBDb1NFTGF5b3V0Lm1vdmVOb2RlcygpO1xyXG59O1xyXG5cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5oYXNBcHByb3hpbWF0aW9uUGVyaW9kUmVhY2hlZCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIGlmKCh0aGlzLnRvdGFsSXRlcmF0aW9ucyAlIDEwMCkgPT09IChTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9QRVJJT0QpKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxufTtcclxuXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUucm90YXRlQVByb2Nlc3MgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgcHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQgPSBbXSAvKkxpc3Q8U2JnblByb2Nlc3NOb2RlPiovO1xyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZQcm9jZXNzTm9kZXMgPSB0aGlzLnByb2Nlc3NOb2RlTGlzdC5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpbmRleDsgaW5kZXg8bnVtT2ZQcm9jZXNzTm9kZXM7IGluZGV4KyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHNiZ25Qcm9jZXNzTm9kZSA9IHRoaXMucHJvY2Vzc05vZGVMaXN0W2luZGV4XTtcclxuICAgICAgICBcclxuICAgICAgICBpZiAoc2JnblByb2Nlc3NOb2RlLmlzUm90YXRpb25OZWNlc3NhcnkoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkLnB1c2goc2JnblByb2Nlc3NOb2RlKTtcclxuICAgICAgICB9ICAgICAgIFxyXG4gICAgfVxyXG5cclxuICAgIC8vIHJhbmRvbSBzZWxlY3Rpb25cclxuICAgIGlmIChwcm9jZXNzTm9kZXNUb0JlUm90YXRlZC5sZW5ndGggPiAwKVxyXG4gICAge1xyXG4gICAgICAgIHZhciByYW5kb21JbmRleCA9IDA7XHJcblxyXG4gICAgICAgIGlmICh0aGlzLnJvdGF0aW9uUmFuZG9taXphdGlvbk1ldGhvZCA9PT0gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJhbmRvbUluZGV4ID0gdGhpcy5yb3VsZXR0ZVdoZWVsU2VsZWN0aW9uKHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkKTtcclxuXHJcbiAgICAgICAgICAgIGlmIChyYW5kb21JbmRleCA9PT0gLTEpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGNvbnNvbGUubG9nKFwiRVJST1I6IG5vIG5vZGVzIGhhdmUgYmVlbiBzZWxlY3RlZCBmb3Igcm90YXRpb25cIik7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmFuZG9tSW5kZXggPSAoTWF0aC5yYW5kb20oKSAqIHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkLmxlbmd0aCk7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICB2YXIgc2JnblByb2Nlc3NOb2RlID0gcHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWRbcmFuZG9tSW5kZXhdO1xyXG4gICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5hcHBseVJvdGF0aW9uKCk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gLy8gcmVzZXQgbmV0IHJvdGF0aW9uYWwgZm9yY2VzIG9uIGFsbCBwcm9jZXNzZXMgZm9yIG5leHQgcm91bmRcclxuICAgIC8vIG5vdCB1c2VkIGJlY2F1c2UgZXZlbiBpZiB0aGUgYW1vdW50IGlzIHNtYWxsLCBzdW1taW5nIHVwIHRoZSBuZXRcclxuICAgIC8vIGZvcmNlIGZyb20gcHJldiBpdGVyYXRpb25zIHlpZWxkIGJldHRlciByZXN1bHRzXHJcbiAgICAvLyBmb3IgKE9iamVjdCBvIDogdGhpcy5nZXRBbGxOb2RlcygpKVxyXG4gICAgLy8ge1xyXG4gICAgLy8gaWYgKG8gaW5zdGFuY2VvZiBTYmduUHJvY2Vzc05vZGUpXHJcbiAgICAvLyAoKFNiZ25Qcm9jZXNzTm9kZSkgbykubmV0Um90YXRpb25hbEZvcmNlID0gMDtcclxuICAgIC8vIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIGl0ZXJhdGVzIG92ZXIgdGhlIHByb2Nlc3Mgbm9kZXMgYW5kIGNoZWNrcyBpZiB0aGVyZSBleGlzdHNcclxuKiBhbm90aGVyIG9yaWVudGF0aW9uIHdoaWNoIG1heGltaXplcyB0aGUgdG90YWwgbnVtYmVyIG9mIHByb3Blcmx5IGVkZ2VzLlxyXG4qIElmIHRoZXJlIGlzLCB0aGUgb3JpZW50YXRpb24gaXMgY2hhbmdlZC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5maW5hbEVuaGFuY2VtZW50ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIG9yaWVudGF0aW9uTGlzdCA9IFtdIC8qTGlzdDxPcmllbnRhdGlvbj4qLztcclxuICAgIHZhciBiZXN0U3RlcFJlc3VsdCA9IDAuMDtcclxuICAgIHZhciBiZXN0T3JpZW50YXRpb24gPSBudWxsIC8qT3JpZW50YXRpb24qLztcclxuICAgIHZhciBzdGVwQXBwcm9wcmlhdGVFZGdlQ250ID0gMC4wO1xyXG4gICAgdmFyIHRvdGFsUHJvcGVyRWRnZXMgPSAwLjA7XHJcbiAgICB2YXIgYW5nbGUgPSAwLjA7XHJcblxyXG4gICAgb3JpZW50YXRpb25MaXN0LnB1c2goU2JnblBETGF5b3V0Lk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUKTtcclxuICAgIG9yaWVudGF0aW9uTGlzdC5wdXNoKFNiZ25QRExheW91dC5PcmllbnRhdGlvbkVudW0uUklHSFRfVE9fTEVGVCk7XHJcbiAgICBvcmllbnRhdGlvbkxpc3QucHVzaChTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLlRPUF9UT19CT1RUT00pO1xyXG4gICAgb3JpZW50YXRpb25MaXN0LnB1c2goU2JnblBETGF5b3V0Lk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QKTtcclxuICAgIFxyXG4gICAgdmFyIG51bU9mUHJvY2Vzc05vZGVzID0gdGhpcy5wcm9jZXNzTm9kZUxpc3QubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaTsgaTxudW1PZlByb2Nlc3NOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzYmduUHJvY2Vzc05vZGUgPSB0aGlzLnByb2Nlc3NOb2RlTGlzdFtpXTtcclxuICAgIFxyXG4gICAgICAgIGJlc3RTdGVwUmVzdWx0ID0gc2JnblByb2Nlc3NOb2RlLnByb3BlckVkZ2VDb3VudDtcclxuICAgICAgICBiZXN0T3JpZW50YXRpb24gPSBudWxsO1xyXG4gICAgICAgIHZhciByZW1lbWJlclByb3BMaXN0ID0gW10vKkxpc3Q8Qm9vbGVhbj4qLztcclxuICAgICAgICB2YXIgYmVzdFByb3BMaXN0ID0gW10vKkxpc3Q8Qm9vbGVhbj4qLztcclxuICAgICAgICBcclxuICAgICAgICB2YXIgbnVtT2ZPcmllbnRhdGlvbnMgPSBvcmllbnRhdGlvbkxpc3QubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGo7IGo8bnVtT2ZPcmllbnRhdGlvbnM7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBvcmllbnQgPSBvcmllbnRhdGlvbkxpc3Rbal07XHJcbiAgICAgICAgICAgIFxyXG4gICAgICAgICAgICBzdGVwQXBwcm9wcmlhdGVFZGdlQ250ID0gMDtcclxuXHJcbiAgICAgICAgICAgIHZhciBpbnB1dFBvcnRUYXJnZXQgPSBzYmduUHJvY2Vzc05vZGUuZmluZFBvcnRUYXJnZXRQb2ludCh0cnVlLCBvcmllbnQpO1xyXG4gICAgICAgICAgICB2YXIgb3V0cHV0UG9ydFRhcmdldCA9IHNiZ25Qcm9jZXNzTm9kZS5maW5kUG9ydFRhcmdldFBvaW50KGZhbHNlLCBvcmllbnQpO1xyXG5cclxuICAgICAgICAgICAgcmVtZW1iZXJQcm9wTGlzdCA9IFtdO1xyXG4gICAgICAgICAgICBcclxuICAgICAgICAgICAgdmFyIG51bU9mQ29uc3VtcHRpb25FZGdlcyA9IHNiZ25Qcm9jZXNzTm9kZS5jb25zdW1wdGlvbkVkZ2VzLmxlbmd0aDtcclxuICAgICAgICAgICAgZm9yICh2YXIgazsgazxudW1PZkNvbnN1bXB0aW9uRWRnZXM7IGsrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdmFyIGVkZ2UgPSBzYmduUHJvY2Vzc05vZGUuY29uc3VtcHRpb25FZGdlc1trXTtcclxuICAgICAgICAgICAgICAgIHZhciBub2RlID0gZWRnZS5nZXRTb3VyY2UoKTtcclxuICAgICAgICAgICAgICAgIGFuZ2xlID0gSUdlb21ldHJ5LmNhbGN1bGF0ZUFuZ2xlKGlucHV0UG9ydFRhcmdldCxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUuaW5wdXRQb3J0LmdldENlbnRlcigpLCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlLmdldENlbnRlcigpKTtcclxuICAgICAgICAgICAgICAgIGlmIChhbmdsZSA8PSBTYmduUERDb25zdGFudHMuQU5HTEVfVE9MRVJBTkNFKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIHN0ZXBBcHByb3ByaWF0ZUVkZ2VDbnQrKztcclxuICAgICAgICAgICAgICAgICAgICByZW1lbWJlclByb3BMaXN0LnB1c2godHJ1ZSk7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgcmVtZW1iZXJQcm9wTGlzdC5wdXNoKGZhbHNlKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBcclxuICAgICAgICAgICAgdmFyIG51bU9mUHJvZHVjdEVkZ2VzID0gc2JnblByb2Nlc3NOb2RlLnByb2R1Y3RFZGdlcy5sZW5ndGg7XHJcbiAgICAgICAgICAgIGZvciAodmFyIGs7IGs8bnVtT2ZQcm9kdWN0RWRnZXM7IGsrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdmFyIGVkZ2UgPSBzYmduUHJvY2Vzc05vZGUucHJvZHVjdEVkZ2VzW2tdO1xyXG4gICAgICAgICAgICAgICAgdmFyIG5vZGUgPSBlZGdlLmdldFRhcmdldCgpO1xyXG4gICAgICAgICAgICAgICAgYW5nbGUgPSBJR2VvbWV0cnkuY2FsY3VsYXRlQW5nbGUob3V0cHV0UG9ydFRhcmdldCxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUub3V0cHV0UG9ydC5nZXRDZW50ZXIoKSwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgbm9kZS5nZXRDZW50ZXIoKSk7XHJcblxyXG4gICAgICAgICAgICAgICAgaWYgKGFuZ2xlIDw9IFNiZ25QRENvbnN0YW50cy5BTkdMRV9UT0xFUkFOQ0UpXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgc3RlcEFwcHJvcHJpYXRlRWRnZUNudCsrO1xyXG4gICAgICAgICAgICAgICAgICAgIHJlbWVtYmVyUHJvcExpc3QucHVzaCh0cnVlKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICByZW1lbWJlclByb3BMaXN0LnB1c2goZmFsc2UpO1xyXG4gICAgICAgICAgICAgICAgfSAgICAgICAgIFxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIFxyXG4gICAgICAgICAgICB2YXIgbnVtT2ZFZmZlY3RvckVkZ2VzID0gc2JnblByb2Nlc3NOb2RlLmVmZmVjdG9yRWRnZXMubGVuZ3RoO1xyXG4gICAgICAgICAgICBmb3IgKHZhciBrOyBrPG51bU9mRWZmZWN0b3JFZGdlczsgaysrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB2YXIgZWRnZSA9IHNiZ25Qcm9jZXNzTm9kZS5lZmZlY3RvckVkZ2VzW2tdO1xyXG4gICAgICAgICAgICAgICAgdmFyIG5vZGUgPSBlZGdlLmdldFNvdXJjZSgpO1xyXG4gICAgICAgICAgICAgICAgYW5nbGUgPSBjYWxjRWZmZWN0b3JBbmdsZShvcmllbnQsIHNiZ25Qcm9jZXNzTm9kZS5nZXRDZW50ZXIoKSwgbm9kZSk7XHJcblxyXG4gICAgICAgICAgICAgICAgaWYgKGFuZ2xlIDw9IFNiZ25QRENvbnN0YW50cy5FRkZFQ1RPUl9BTkdMRV9UT0xFUkFOQ0UpXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgc3RlcEFwcHJvcHJpYXRlRWRnZUNudCsrO1xyXG4gICAgICAgICAgICAgICAgICAgIHJlbWVtYmVyUHJvcExpc3QucHVzaCh0cnVlKTtcclxuXHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgcmVtZW1iZXJQcm9wTGlzdC5wdXNoKGZhbHNlKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgaWYgKHN0ZXBBcHByb3ByaWF0ZUVkZ2VDbnQgPiBiZXN0U3RlcFJlc3VsdClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgYmVzdFN0ZXBSZXN1bHQgPSBzdGVwQXBwcm9wcmlhdGVFZGdlQ250O1xyXG4gICAgICAgICAgICAgICAgYmVzdE9yaWVudGF0aW9uID0gb3JpZW50O1xyXG4gICAgICAgICAgICAgICAgYmVzdFByb3BMaXN0ID0gcmVtZW1iZXJQcm9wTGlzdDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICB0b3RhbFByb3BlckVkZ2VzICs9IGJlc3RTdGVwUmVzdWx0O1xyXG5cclxuICAgICAgICAvLyBpdCBtZWFucyBhIGJldHRlciBwb3NpdGlvbiBoYXMgYmVlbiBmb3VuZFxyXG4gICAgICAgIGlmIChiZXN0U3RlcFJlc3VsdCA+IHNiZ25Qcm9jZXNzTm9kZS5wcm9wZXJFZGdlQ291bnQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUuc2V0T3JpZW50YXRpb24oYmVzdE9yaWVudGF0aW9uKTtcclxuICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLnByb3BlckVkZ2VDb3VudCA9IGJlc3RTdGVwUmVzdWx0O1xyXG5cclxuICAgICAgICAgICAgLy8gbWFyayBlZGdlcyB3aXRoIGJlc3Qga25vd24gY29uZmlndXJhdGlvbiB2YWx1ZXNcclxuICAgICAgICAgICAgZm9yICh2YXIgbSA9IDA7IG0gPCBzYmduUHJvY2Vzc05vZGUuY29uc3VtcHRpb25FZGdlcy5sZW5ndGg7IG0rKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLmNvbnN1bXB0aW9uRWRnZXNbaV0uaXNQcm9wZXJseU9yaWVudGVkID0gYmVzdFByb3BMaXN0W2ldO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGZvciAodmFyIG4gPSAwOyBuIDwgc2JnblByb2Nlc3NOb2RlLnByb2R1Y3RFZGdlcy5sZW5ndGg7IG4rKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLnByb2R1Y3RFZGdlc1tpXS5pc1Byb3Blcmx5T3JpZW50ZWQgPSBiZXN0UHJvcExpc3RbaSArIHNiZ25Qcm9jZXNzTm9kZS5jb25zdW1wdGlvbkVkZ2VzLmxlbmd0aF07XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZm9yICh2YXIgbyA9IDA7IG8gPCBzYmduUHJvY2Vzc05vZGUuZWZmZWN0b3JFZGdlcy5zaXplKCk7IG8rKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5lZmZlY3RvckVkZ2VzW2ldLmlzUHJvcGVybHlPcmllbnRlZCA9IFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgYmVzdFByb3BMaXN0W2kgKyBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAoc2JnblByb2Nlc3NOb2RlLmNvbnN1bXB0aW9uRWRnZXMubGVuZ3RoICsgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5wcm9kdWN0RWRnZXMubGVuZ3RoKV07XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50ID0gdG90YWxQcm9wZXJFZGdlcztcclxuICAgIHRoaXMuZW5oYW5jZWRSYXRpbyA9IHRvdGFsUHJvcGVyRWRnZXMgLyB0b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZDtcclxuXHJcbiAgICB2YXIgbnVtT2ZQcm9jZXNzTm9kZXMgPSB0aGlzLnByb2Nlc3NOb2RlTGlzdC5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpOyBpPG51bU9mUHJvY2Vzc05vZGVzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdG90YWxFZmZDb3VudCArPSB0aGlzLnByb2Nlc3NOb2RlTGlzdFtpXS5lZmZlY3RvckVkZ2VzLmxlbmd0aDtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIElmIGEgcHJvY2VzcyBub2RlIGhhcyBoaWdoZXIgbmV0Um90YXRpb25hbEZvcmNlLCBpdCBoYXMgbW9yZSBjaGFuY2UgdG8gYmVcclxuKiByb3RhdGVkXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUucm91bGV0dGVXaGVlbFNlbGVjdGlvbiA9IGZ1bmN0aW9uIChcclxuICAgICAgICAgICAgICAgLypBcnJheUxpc3Q8U2JnblByb2Nlc3NOb2RlPiovIHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkKVxyXG57XHJcbiAgICB2YXIgcmFuZG9tTnVtYmVyID0gTWF0aC5yYW5kb20oKTtcclxuICAgIHZhciBmaXRuZXNzVmFsdWVzID0gW10gLypwcm9jZXNzTm9kZXNUb0JlUm90YXRlZC5zaXplKCldKi87XHJcbiAgICB2YXIgdG90YWxTdW0gPSAwO1xyXG4gICAgdmFyIHN1bU9mUHJvYmFiaWxpdGllcyA9IDA7XHJcbiAgICB2YXIgaSA9IDA7XHJcblxyXG4gICAgdmFyIG51bU9mUHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQgPSBwcm9jZXNzTm9kZXNUb0JlUm90YXRlZC5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBqOyBqPG51bU9mUHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQ7IGorKylcclxuICAgIHtcclxuICAgICAgICB0b3RhbFN1bSArPSBNYXRoLmFicyhwcm9jZXNzTm9kZXNUb0JlUm90YXRlZFtqXS5uZXRSb3RhdGlvbmFsRm9yY2UpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIG5vcm1hbGl6ZSBhbGwgYmV0d2VlbiAwLi4xXHJcbiAgICB2YXIgbnVtT2ZQcm9jZXNzTm9kZXNUb0JlUm90YXRlZCA9IHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGo7IGo8bnVtT2ZQcm9jZXNzTm9kZXNUb0JlUm90YXRlZDsgaisrKVxyXG4gICAge1xyXG4gICAgICAgIGZpdG5lc3NWYWx1ZXNbaV0gPSBcclxuICAgICAgICAgICAgICAgIHN1bU9mUHJvYmFiaWxpdGllcyArIFxyXG4gICAgICAgICAgICAgICAgKE1hdGguYWJzKHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkW2pdLm5ldFJvdGF0aW9uYWxGb3JjZSkgLyBcclxuICAgICAgICAgICAgICAgICAgICB0b3RhbFN1bSk7XHJcblxyXG4gICAgICAgIHN1bU9mUHJvYmFiaWxpdGllcyA9IGZpdG5lc3NWYWx1ZXNbaV07XHJcbiAgICAgICAgaSsrO1xyXG4gICAgfVxyXG4gICAgICAgIFxyXG4gICAgaWYgKHJhbmRvbU51bWJlciA8IGZpdG5lc3NWYWx1ZXNbMF0pXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIDA7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgZm9yICh2YXIgaiA9IDA7IGogPCBmaXRuZXNzVmFsdWVzLmxlbmd0aCAtIDE7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICgocmFuZG9tTnVtYmVyID49IGZpdG5lc3NWYWx1ZXNbal0pICYmIFxyXG4gICAgICAgICAgICAgICAgKHJhbmRvbU51bWJlciA8IGZpdG5lc3NWYWx1ZXNbaisxXSkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiBqICsgMTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gLTE7XHJcbn07XHJcblxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmNhbGNFZmZlY3RvckFuZ2xlID0gZnVuY3Rpb24gKFxyXG4gICAgICAgIC8qT3JpZW50YXRpb24qLyBvcmllbnQsIFxyXG4gICAgICAgIC8qUG9pbnREKi8gICAgICBjZW50ZXJQdCxcclxuICAgICAgICAvKkNvU0VOb2RlKi8gICAgZWZmKVxyXG57XHJcbiAgICB2YXIgaWRlYWxFZGdlTGVuZ3RoID0gdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICB2YXIgdGFyZ2V0UG50ID0gbmV3IFBvaW50RCgpO1xyXG4gICAgdmFyIGNlbnRlclBudCA9IGNlbnRlclB0O1xyXG5cclxuICAgIC8vIGZpbmQgdGFyZ2V0IHBvaW50XHJcbiAgICBpZiAob3JpZW50ID09PSBTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQgfHwgXHJcbiAgICAgICAgb3JpZW50ID09PSBTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UG50LnggPSBjZW50ZXJQbnQueDtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJZKCkgPiBjZW50ZXJQbnQueSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRhcmdldFBudC55ID0gY2VudGVyUG50LnkgKyBpZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRhcmdldFBudC55ID0gY2VudGVyUG50LnkgLSBpZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAob3JpZW50ID09PSBTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLkJPVFRPTV9UT19UT1AgfHwgXHJcbiAgICAgICAgICAgICBvcmllbnQgPT09IFNiZ25QRExheW91dC5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTSlcclxuICAgIHtcclxuICAgICAgICB0YXJnZXRQbnQueSA9IGNlbnRlclBudC55O1xyXG5cclxuICAgICAgICBpZiAoZWZmLmdldENlbnRlclgoKSA+IGNlbnRlclBudC54KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnggPSBjZW50ZXJQbnQueCArIGlkZWFsRWRnZUxlbmd0aDtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnggPSBjZW50ZXJQbnQueCAtIGlkZWFsRWRnZUxlbmd0aDtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgdmFyIGFuZ2xlID0gSUdlb21ldHJ5LmNhbGN1bGF0ZUFuZ2xlKHRhcmdldFBudCwgY2VudGVyUG50LCBlZmYuZ2V0Q2VudGVyKCkpO1xyXG5cclxuICAgIHJldHVybiBhbmdsZTtcclxufTtcclxuXHJcbi8qKlxyXG4qIFJlY3Vyc2l2ZWx5IGNhbGN1bGF0ZSBpZiB0aGUgbm9kZSBvciBpdHMgY2hpbGQgbm9kZXMgaGF2ZSBhbnkgZWRnZXMgdG9cclxuKiBvdGhlciBub2Rlcy4gUmV0dXJuIHRoZSB0b3RhbCBudW1iZXIgb2YgZWRnZXMuXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY0dyYXBoRGVncmVlID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIHBhcmVudE5vZGUpXHJcbntcclxuICAgIHZhciBkZWdyZWUgPSAwO1xyXG4gICAgaWYgKHBhcmVudE5vZGUuZ2V0Q2hpbGQoKSA9PSBudWxsKVxyXG4gICAge1xyXG4gICAgICAgIGRlZ3JlZSA9IHBhcmVudE5vZGUuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgcmV0dXJuIGRlZ3JlZTtcclxuICAgIH1cclxuXHJcbiAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IHBhcmVudE5vZGUuZ2V0Q2hpbGQoKS5nZXROb2RlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNoaWxkcmVuOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgZGVncmVlID0gZGVncmVlICsgcGFyZW50Tm9kZS5nZXRFZGdlcygpLmxlbmd0aFxyXG4gICAgICAgICAgICAgICAgICAgICAgICArIHRoaXMuY2FsY0dyYXBoRGVncmVlKHBhcmVudE5vZGUuZ2V0Q2hpbGQoKS5nZXROb2RlcygpW2ldKTtcclxuICAgIH1cclxuICAgIFxyXG4gICAgcmV0dXJuIGRlZ3JlZTtcclxufTtcclxuXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUucmVjYWxjUHJvcGVybHlPcmllbnRlZEVkZ2VzID0gZnVuY3Rpb24gKC8qYm9vbGVhbiovIGlzTGFzdEl0ZXJhdGlvbilcclxue1xyXG4gICAgdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50ID0gMC4wO1xyXG4gICAgdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZCA9IDA7XHJcbiAgICBcclxuICAgIHZhciBudW1PZlByb2Nlc3NOb2RlcyA9IHRoaXMucHJvY2Vzc05vZGVMaXN0Lmxlbmd0aDtcclxuICAgIGZvcih2YXIgaT0wOyBpPG51bU9mUHJvY2Vzc05vZGVzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHNiZ25Qcm9jZXNzTm9kZSA9IHRoaXMucHJvY2Vzc05vZGVMaXN0W2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5jYWxjUHJvcGVybHlPcmllbnRlZEVkZ2VzKCk7XHJcbiAgICAgICAgdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50ICs9IHNiZ25Qcm9jZXNzTm9kZS5wcm9wZXJFZGdlQ291bnQ7XHJcbiAgICAgICAgdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZCArPSBcclxuICAgICAgICAgICAgICAgIChzYmduUHJvY2Vzc05vZGUuY29uc3VtcHRpb25FZGdlcy5sZW5ndGggKyBcclxuICAgICAgICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUucHJvZHVjdEVkZ2VzLmxlbmd0aCArIFxyXG4gICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5lZmZlY3RvckVkZ2VzLmxlbmd0aCk7XHJcbiAgICAgICAgdGhpcy5zdWNjZXNzUmF0aW8gPSBcclxuICAgICAgICAgICAgICAgIHRoaXMucHJvcGVybHlPcmllbnRlZEVkZ2VDb3VudCAvIHRoaXMudG90YWxFZGdlQ291bnRUb0JlT3JpZW50ZWQ7XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBmaW5kcyBhbGwgdGhlIHplcm8gZGVncmVlIG5vZGVzIGluIHRoZSBncmFwaCB3aGljaCBhcmUgbm90XHJcbiogb3duZWQgYnkgYSBjb21wbGV4IG5vZGUuIFplcm8gZGVncmVlIG5vZGVzIGF0IGVhY2ggbGV2ZWwgYXJlIGdyb3VwZWRcclxuKiB0b2dldGhlciBhbmQgcGxhY2VkIGluc2lkZSBhIGR1bW15IGNvbXBsZXggdG8gcmVkdWNlIGJvdW5kcyBvZiByb290XHJcbiogZ3JhcGguXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuZ3JvdXBaZXJvRGVncmVlTWVtYmVycyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBjaGlsZENvbXBsZXhNYXAgPSBuZXcgSGFzaE1hcCgpOy8qU2JnblBETm9kZSwgTEdyYXBoKi9cclxuICAgIFxyXG4gICAgdmFyIG51bU9mR3JhcGhzID0gdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZHcmFwaHM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgb3duZXJHcmFwaCA9IHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKClbaV07XHJcbiAgICAgICAgdmFyIHplcm9EZWdyZWVOb2RlcyA9IFtdOyAvKkFycmF5TGlzdDxTYmduUEROb2RlPiovXHJcbiAgICAgICAgXHJcbiAgICAgICAgLy8gZG8gbm90IHByb2Nlc3MgY29tcGxleCBub2RlcyAodGhlaXIgbWVtYmVycyBhcmUgYWxyZWFkeSBvd25lZClcclxuICAgICAgICBpZiAoKG93bmVyR3JhcGguZ2V0UGFyZW50KCkudHlwZSAhPT0gbnVsbCkgJiYgXHJcbiAgICAgICAgICAgIChvd25lckdyYXBoLmdldFBhcmVudCgpLmlzQ29tcGxleCgpKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuICAgICAgICBcclxuICAgICAgICB2YXIgbnVtT2ZOb2RlcyA9IG93bmVyR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgaj0wOyBqPG51bU9mTm9kZXM7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBub2RlID0gb3duZXJHcmFwaC5nZXROb2RlcygpW2pdO1xyXG5cclxuICAgICAgICAgICAgaWYgKHRoaXMuY2FsY0dyYXBoRGVncmVlKG5vZGUpID09PSAwKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB6ZXJvRGVncmVlTm9kZXMucHVzaChub2RlKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKHplcm9EZWdyZWVOb2Rlcy5sZW5ndGggPiAxKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgLy8gY3JlYXRlIGEgbmV3IGR1bW15IGNvbXBsZXhcclxuICAgICAgICAgICAgdmFyIGNvbXBsZXggPSB0aGlzLm5ld05vZGUobnVsbCk7XHJcbiAgICAgICAgICAgIGNvbXBsZXgudHlwZSA9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYO1xyXG4gICAgICAgICAgICBjb21wbGV4LmxhYmVsID0gXCJEdW1teUNvbXBsZXhfXCIgKyBvd25lckdyYXBoLmdldFBhcmVudCgpLmxhYmVsO1xyXG5cclxuICAgICAgICAgICAgb3duZXJHcmFwaC5hZGQoY29tcGxleCk7XHJcblxyXG4gICAgICAgICAgICB2YXIgY2hpbGRHcmFwaCA9IG5ld0dyYXBoKG51bGwpO1xyXG4gICAgICAgICAgICBcclxuICAgICAgICAgICAgdmFyIG51bU9mWmVyb0RlZ3JlZU5vZGUgPSB6ZXJvRGVncmVlTm9kZXMubGVuZ3RoO1xyXG4gICAgICAgICAgICBmb3IgKHZhciBqPTA7IGo8bnVtT2ZaZXJvRGVncmVlTm9kZTsgaisrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB2YXIgemVyb05vZGUgPSB6ZXJvRGVncmVlTm9kZXNbal07XHJcbiAgICAgICAgICAgICAgICBvd25lckdyYXBoLnJlbW92ZSh6ZXJvTm9kZSk7XHJcbiAgICAgICAgICAgICAgICBjaGlsZEdyYXBoLmFkZCh6ZXJvTm9kZSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIHRoaXMuZHVtbXlDb21wbGV4TGlzdC5wdXNoKGNvbXBsZXgpO1xyXG4gICAgICAgICAgICBjaGlsZENvbXBsZXhNYXAucHV0KGNvbXBsZXgsIGNoaWxkR3JhcGgpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIFxyXG4gICAgdmFyIG51bU9mQ29tcGxleE5vZGVzID0gdGhpcy5kdW1teUNvbXBsZXhMaXN0Lmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNvbXBsZXhOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLmFkZChjaGlsZENvbXBsZXhNYXAuZ2V0KGNvbXBsZXgpLCB0aGlzLmR1bW15Q29tcGxleExpc3RbaV0pO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnVwZGF0ZUJvdW5kcygpO1xyXG5cclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24oKTtcclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsRWRnZXMoKTtcclxuICAgIHRoaXMuY2FsY3VsYXRlTm9kZXNUb0FwcGx5R3Jhdml0YXRpb25UbygpO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY3JlYXRlcyB0d28gcG9ydCBub2RlcyBhbmQgYSBjb21wb3VuZCBmb3IgZWFjaCBwcm9jZXNzIG5vZGVzXHJcbiogYW5kIGFkZHMgdGhlbSB0byBncmFwaC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jcmVhdGVQb3J0Tm9kZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgbnVtT2ZOb2RlcyA9IHRoaXMuZ2V0QWxsTm9kZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBvcmlnaW5hbFByb2Nlc3NOb2RlID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAob3JpZ2luYWxQcm9jZXNzTm9kZS50eXBlID09PSBTYmduUERDb25zdGFudHMuUFJPQ0VTUylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBvd25lckdyYXBoID0gb3JpZ2luYWxQcm9jZXNzTm9kZS5nZXRPd25lcigpO1xyXG5cclxuICAgICAgICAgICAgLy8gY3JlYXRlIG5ldyBub2RlcyBhbmQgZ3JhcGhzXHJcbiAgICAgICAgICAgIHZhciBwcm9jZXNzTm9kZSA9IG5ld1Byb2Nlc3NOb2RlKG51bGwpO1xyXG4gICAgICAgICAgICB2YXIgaW5wdXRQb3J0ICAgPSBuZXdQb3J0Tm9kZShudWxsLCBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVCk7XHJcbiAgICAgICAgICAgIHZhciBvdXRwdXRQb3J0ICA9IG5ld1BvcnROb2RlKG51bGwsIFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVCk7XHJcblxyXG4gICAgICAgICAgICAvLyBjcmVhdGUgYSBkdW1teSBjb21wb3VuZFxyXG4gICAgICAgICAgICB2YXIgY29tcG91bmROb2RlID0gbmV3Tm9kZShudWxsKTtcclxuICAgICAgICAgICAgY29tcG91bmROb2RlLnR5cGUgPSBTYmduUERDb25zdGFudHMuRFVNTVlfQ09NUE9VTkQ7XHJcblxyXG4gICAgICAgICAgICAvLyBhZGQgbGFiZWxzXHJcbiAgICAgICAgICAgIGNvbXBvdW5kTm9kZS5sYWJlbCA9IFwiRHVtbXlDb21wb3VuZF9cIiArIG9yaWdpbmFsUHJvY2Vzc05vZGUubGFiZWw7XHJcbiAgICAgICAgICAgIGlucHV0UG9ydC5sYWJlbCA9IFwiSW5wdXRQb3J0X1wiICsgb3JpZ2luYWxQcm9jZXNzTm9kZS5sYWJlbDtcclxuICAgICAgICAgICAgb3V0cHV0UG9ydC5sYWJlbCA9IFwiT3V0cHV0UG9ydF9cIiArIG9yaWdpbmFsUHJvY2Vzc05vZGUubGFiZWw7XHJcblxyXG4gICAgICAgICAgICAvLyBjcmVhdGUgY2hpbGQgZ3JhcGggKD0gMnBvcnQrcHJvY2VzcykgdG8gYmUgc2V0IGFzIGNoaWxkIHRvXHJcbiAgICAgICAgICAgIC8vIGR1bW15IGNvbXBvdW5kXHJcbiAgICAgICAgICAgIHZhciBjaGlsZEdyYXBoID0gbmV3R3JhcGgobnVsbCk7XHJcbiAgICAgICAgICAgIG93bmVyR3JhcGguYWRkKHByb2Nlc3NOb2RlKTtcclxuXHJcbiAgICAgICAgICAgIC8vIGNvbnZlcnQgdGhlIHByb2Nlc3Mgbm9kZSB0byBTYmduUHJvY2Vzc05vZGVcclxuICAgICAgICAgICAgcHJvY2Vzc05vZGUuY29weUZyb21TQkdOUEROb2RlKG9yaWdpbmFsUHJvY2Vzc05vZGUsXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpKTtcclxuXHJcbiAgICAgICAgICAgIHByb2Nlc3NOb2RlLmNvbm5lY3ROb2Rlcyhjb21wb3VuZE5vZGUsIGlucHV0UG9ydCwgb3V0cHV0UG9ydCk7XHJcblxyXG4gICAgICAgICAgICAvLyBjcmVhdGUgcmlnaWQgZWRnZXMsIGNoYW5nZSBlZGdlIGNvbm5lY3Rpb25zXHJcbiAgICAgICAgICAgIHByb2Nlc3NOb2RlLnJlY29ubmVjdEVkZ2VzKGlkZWFsRWRnZUxlbmd0aCk7XHJcblxyXG4gICAgICAgICAgICB2YXIgcmlnaWRUb1Byb2R1Y3Rpb24gPSBuZXdSaWdpZEVkZ2UobnVsbCk7XHJcbiAgICAgICAgICAgIHJpZ2lkVG9Qcm9kdWN0aW9uLmxhYmVsID0gXCJcIlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKyAodGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsRWRnZXMoKS5sZW5ndGggKyAxKTtcclxuXHJcbiAgICAgICAgICAgIHZhciByaWdpZFRvQ29uc3VtcHRpb24gPSBuZXdSaWdpZEVkZ2UobnVsbCk7XHJcbiAgICAgICAgICAgIHJpZ2lkVG9Db25zdW1wdGlvbi5sYWJlbCA9IFwiXCJcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICsgKHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEFsbEVkZ2VzKCkubGVuZ3RoICsgMik7XHJcblxyXG4gICAgICAgICAgICBvd25lckdyYXBoLnJlbW92ZShwcm9jZXNzTm9kZSk7XHJcblxyXG4gICAgICAgICAgICAvLyBvcmdhbml6ZSBjaGlsZCBncmFwaFxyXG4gICAgICAgICAgICBjaGlsZEdyYXBoLmFkZChwcm9jZXNzTm9kZSk7XHJcbiAgICAgICAgICAgIGNoaWxkR3JhcGguYWRkKGlucHV0UG9ydCk7XHJcbiAgICAgICAgICAgIGNoaWxkR3JhcGguYWRkKG91dHB1dFBvcnQpO1xyXG4gICAgICAgICAgICBjaGlsZEdyYXBoLmFkZChyaWdpZFRvUHJvZHVjdGlvbiwgaW5wdXRQb3J0LCBwcm9jZXNzTm9kZSk7XHJcbiAgICAgICAgICAgIGNoaWxkR3JhcGguYWRkKHJpZ2lkVG9Db25zdW1wdGlvbiwgb3V0cHV0UG9ydCwgcHJvY2Vzc05vZGUpO1xyXG5cclxuICAgICAgICAgICAgLy8gb3JnYW5pemUgdGhlIGNvbXBvdW5kIG5vZGVcclxuICAgICAgICAgICAgY29tcG91bmROb2RlLnNldE93bmVyKG93bmVyR3JhcGgpO1xyXG4gICAgICAgICAgICBjb21wb3VuZE5vZGUuc2V0Q2VudGVyKHByb2Nlc3NOb2RlLmdldENlbnRlclgoKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHByb2Nlc3NOb2RlLmdldENlbnRlclkoKSk7XHJcbiAgICAgICAgICAgIG93bmVyR3JhcGguYWRkKGNvbXBvdW5kTm9kZSk7XHJcbiAgICAgICAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLmFkZChjaGlsZEdyYXBoLCBjb21wb3VuZE5vZGUpO1xyXG5cclxuICAgICAgICAgICAgLy8gcmVtb3ZlIHRoZSBvcmlnaW5hbCBwcm9jZXNzIG5vZGVcclxuICAgICAgICAgICAgb3duZXJHcmFwaC5yZW1vdmUob3JpZ2luYWxQcm9jZXNzTm9kZSk7XHJcblxyXG4gICAgICAgICAgICB0aGlzLnByb2Nlc3NOb2RlTGlzdC5wdXNoKHByb2Nlc3NOb2RlKTtcclxuICAgICAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIudXBkYXRlQm91bmRzKCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8vIHJlc2V0IHRoZSB0b3BvbG9neVxyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxOb2RlcygpO1xyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbigpO1xyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxFZGdlcygpO1xyXG5cclxuICAgIHRoaXMuY2FsY3VsYXRlTm9kZXNUb0FwcGx5R3Jhdml0YXRpb25UbygpO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY2hlY2tzIHdoZXRoZXIgdGhlcmUgZXhpc3RzIGFueSBwcm9jZXNzIG5vZGVzIGluIHRoZSBncmFwaC5cclxuKiBJZiB0aGVyZSBleGlzdCBhbnkgcHJvY2VzcyBub2RlcyBpdCBpcyBhc3N1bWVkIHRoYXQgdGhlIGdpdmVuIGdyYXBoXHJcbiogcmVzcGVjdHMgb3VyIHN0cnVjdHVyZS5cclxuKiBcclxuKiBNb3N0IGxpa2VseTogdGhpcyBtZXRob2QgZG9lcyBub3Qgd29yayBwcm9wZXJseS4gTmV2ZXIgaGFkIGFueSBpbnB1dCB0b1xyXG4qIHRlc3QuIE5vdCBjb21wbGV0ZS5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5hcmVQb3J0Tm9kZXNDcmVhdGVkID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGZsYWcgPSBmYWxzZTtcclxuXHJcbiAgICAvLyBpZiB0aGVyZSBhcmUgYW55IHByb2Nlc3Mgbm9kZXMsIGNoZWNrIGZvciBwb3J0IG5vZGVzXHJcbiAgICB2YXIgbnVtT2ZOb2RlcyA9IHRoaXMuZ2V0QWxsTm9kZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzYmduUEROb2RlID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIGlmIChzYmduUEROb2RlLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5QUk9DRVNTKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgZmxhZyA9IHRydWU7XHJcbiAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIFxyXG4gICAgLy8gaWYgdGhlcmUgYXJlIG5vIHByb2Nlc3Mgbm9kZXMsIG5vIG5lZWQgdG8gY2hlY2sgZm9yIHBvcnQgbm9kZXNcclxuICAgIGlmICghZmxhZylcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICAvLyBjaGVjayBmb3IgdGhlIHBvcnQgbm9kZXMuIGlmIGFueSBmb3VuZCwgcmV0dXJuIHRydWUuXHJcbiAgICAgICAgdmFyIG51bU9mTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGk9MDsgaTxudW1PZk5vZGVzOyBpKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgc2JnblBETm9kZSA9IHRoaXMuZ2V0QWxsTm9kZXMoKVtpXTtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIGlmIChzYmduUEROb2RlLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgc2JnblBETm9kZS50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgXHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBpcyB1c2VkIHRvIHJlbW92ZSB0aGUgZHVtbXkgY29tcG91bmRzIChwcmV2aW91c2x5IGNyZWF0ZWQgZm9yXHJcbiogZWFjaCBwcm9jZXNzIG5vZGUpIGZyb20gdGhlIGdyYXBoLlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLnJlbW92ZUR1bW15Q29tcG91bmRzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIG51bU9mUHJvY2Vzc05vZGVzID0gdGhpcy5wcm9jZXNzTm9kZUxpc3QubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaT0wOyBpPG51bU9mUHJvY2Vzc05vZGVzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHByb2Nlc3NOb2RlID0gdGhpcy5wcm9jZXNzTm9kZUxpc3RbaV07XHJcbiAgICAgICAgdmFyIGR1bW15Tm9kZSA9IHByb2Nlc3NOb2RlLnBhcmVudENvbXBvdW5kO1xyXG4gICAgICAgIHZhciBjaGlsZEdyYXBoID0gZHVtbXlOb2RlLmdldENoaWxkKCk7XHJcbiAgICAgICAgdmFyIG93bmVyID0gZHVtbXlOb2RlLmdldE93bmVyKCk7XHJcblxyXG4gICAgICAgIC8vIGFkZCBjaGlsZHJlbiB0byBvcmlnaW5hbCBwYXJlbnRcclxuICAgICAgICB2YXIgbnVtT2ZDaGlsZE5vZGVzID0gY2hpbGRHcmFwaC5nZXROb2RlcygpLmxlbmd0aDtcclxuICAgICAgICBmb3IgKHZhciBqPTA7IGo8bnVtT2ZDaGlsZE5vZGVzOyBqKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBvd25lci5hZGQoY2hpbGRHcmFwaC5nZXROb2RlcygpW2pdKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHZhciBudW1PZkNoaWxkRWRnZXMgPSBjaGlsZEdyYXBoLmdldEVkZ2VzKCkubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGo9MDsgajxudW1PZkNoaWxkRWRnZXM7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBlZGdlID0gY2hpbGRHcmFwaC5nZXRFZGdlcygpW2pdXHJcbiAgICAgICAgICAgIG93bmVyLmFkZChlZGdlLCBlZGdlLmdldFNvdXJjZSgpLCBlZGdlLmdldFRhcmdldCgpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgXHJcbiAgICAgICAgLy8gYWRkIGVmZmVjdG9ycyAvIHJlbWFpbmluZyBlZGdlcyBiYWNrIHRvIHRoZSBwcm9jZXNzXHJcbiAgICAgICAgZm9yICh2YXIgaiA9IDA7IGogPCBkdW1teU5vZGUuZ2V0RWRnZXMoKS5sZW5ndGg7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBlZGdlID0gZHVtbXlOb2RlLmdldEVkZ2VzKClbal07XHJcblxyXG4gICAgICAgICAgICBkdW1teU5vZGUuZ2V0RWRnZXMoKS5zcGxpY2UoaiwgMSk7XHJcbiAgICAgICAgICAgIGVkZ2Uuc2V0VGFyZ2V0KHByb2Nlc3NOb2RlKTtcclxuICAgICAgICAgICAgcHJvY2Vzc05vZGUuZ2V0RWRnZXMoKS5wdXNoKGVkZ2UpO1xyXG4gICAgICAgICAgICBqLS07XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyByZW1vdmUgdGhlIGdyYXBoXHJcbiAgICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5zcGxpY2UodGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5pbmRleE9mKGNoaWxkR3JhcGgpLCAxKTtcclxuICAgICAgICBkdW1teU5vZGUuc2V0Q2hpbGQobnVsbCk7XHJcbiAgICAgICAgb3duZXIucmVtb3ZlKGR1bW15Tm9kZSk7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5yZXNldEFsbE5vZGVzKCk7XHJcbiAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnJlc2V0QWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24oKTtcclxuICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkucmVzZXRBbGxFZGdlcygpO1xyXG4gICAgdGhpcy5jYWxjdWxhdGVOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvblRvKCk7XHJcbn07XHJcblxyXG5cclxuLy8gKioqKioqKioqKioqKioqKioqKioqIFNFQ1RJT04gOiBUSUxJTkcgTUVUSE9EUyAqKioqKioqKioqKioqKioqKioqKipcclxuXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2xlYXJDb21wbGV4ID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIGNvbXApXHJcbntcclxuICAgIHZhciBwYWNrID0gbnVsbDsgLyogTWVtYmVyUGFjayAqL1xyXG4gICAgdmFyIGNoaWxkR3IgPSBjb21wLmdldENoaWxkKCk7IC8qIExHcmFwaCAqL1xyXG4gICAgdGhpcy5jaGlsZEdyYXBoTWFwLnB1dChjb21wLCBjaGlsZEdyKTtcclxuXHJcbiAgICBpZiAoY2hpbGRHciA9PSBudWxsKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybjtcclxuICAgIH1cclxuICAgIFxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PSBTYmduUERMYXlvdXQuRGVmYXVsdENvbXBhY3Rpb25BbGdvcml0aG1FbnVtLlBPTFlPTUlOT19QQUNLSU5HKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuYXBwbHlQb2x5b21pbm8oY29tcCk7XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmICh0aGlzLmNvbXBhY3Rpb25NZXRob2QgPT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5USUxJTkcpXHJcbiAgICB7XHJcbiAgICAgICAgcGFjayA9IG5ldyBNZW1iZXJQYWNrKGNoaWxkR3IpO1xyXG4gICAgICAgIHRoaXMubWVtYmVyUGFja01hcC5wdXQoY29tcCwgcGFjayk7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHRoaXMuZHVtbXlDb21wbGV4TGlzdC5pbmNsdWRlcyhjb21wKSlcclxuICAgIHtcclxuICAgICAgICBmb3IgKHZhciBpPTA7IGk8Y29tcC5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoOyBpKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBjbGVhckR1bW15Q29tcGxleEdyYXBocyhjb21wLmdldENoaWxkKCkuZ2V0Tm9kZXMoKVtpXSk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHZhciByZW1JbmRleCA9IHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkuaW5kZXhPZihjaGlsZEdyKTtcclxuICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkuc3BsaWNlKHJlbUluZGV4LCAxKTtcclxuICAgIGNvbXAuc2V0Q2hpbGQobnVsbCk7XHJcblxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PSBTYmduUERMYXlvdXQuRGVmYXVsdENvbXBhY3Rpb25BbGdvcml0aG1FbnVtLlRJTElORylcclxuICAgIHtcclxuICAgICAgICBjb21wLnNldFdpZHRoKHBhY2suZ2V0V2lkdGgoKSk7XHJcbiAgICAgICAgY29tcC5zZXRIZWlnaHQocGFjay5nZXRIZWlnaHQoKSk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gUmVkaXJlY3QgdGhlIGVkZ2VzIG9mIGNvbXBsZXggbWVtYmVycyB0byB0aGUgY29tcGxleC5cclxuICAgIGlmIChjaGlsZEdyICE9IG51bGwpXHJcbiAgICB7XHJcbiAgICAgICAgZm9yICh2YXIgaT0wOyBpPGNoaWxkR3IuZ2V0Tm9kZXMoKS5sZW5ndGg7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBjaE5kID0gY2hpbGRHci5nZXROb2RlcygpW2ldO1xyXG5cclxuICAgICAgICAgICAgZm9yICh2YXIgaj0wOyBqPGNoTmQuZ2V0RWRnZXMoKS5sZW5ndGg7IGorKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdmFyIGVkZ2UgPSBjaE5kLmdldEVkZ2VzKClbal07XHJcbiAgICAgICAgICAgICAgICBpZiAoZWRnZS5nZXRTb3VyY2UoKSA9PSBjaE5kKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGNoTmQuZ2V0RWRnZXMoKS5zcGxpY2UoY2hOZC5nZXRFZGdlcygpLmluZGV4T2YoZWRnZSksIDEpO1xyXG4gICAgICAgICAgICAgICAgICAgIGVkZ2Uuc2V0U291cmNlKGNvbXApO1xyXG4gICAgICAgICAgICAgICAgICAgIGNvbXAuZ2V0RWRnZXMoKS5wdXNoKGVkZ2UpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgZWxzZSBpZiAoZWRnZS5nZXRUYXJnZXQoKSA9PSBjaE5kKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGNoTmQuZ2V0RWRnZXMoKS5zcGxpY2UoY2hOZC5nZXRFZGdlcygpLmluZGV4T2YoZWRnZSksIDEpO1xyXG4gICAgICAgICAgICAgICAgICAgIGVkZ2Uuc2V0VGFyZ2V0KGNvbXApO1xyXG4gICAgICAgICAgICAgICAgICAgIGNvbXAuZ2V0RWRnZXMoKS5wdXNoKGVkZ2UpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2Qgc2VhcmNoZWQgdW5tYXJrZWQgY29tcGxleCBub2RlcyByZWN1cnNpdmVseSwgYmVjYXVzZSB0aGV5IG1heVxyXG4qIGNvbnRhaW4gY29tcGxleCBjaGlsZHJlbi4gQWZ0ZXIgdGhlIG9yZGVyIGlzIGZvdW5kLCBjaGlsZCBncmFwaHMgb2YgZWFjaFxyXG4qIGNvbXBsZXggbm9kZSBhcmUgY2xlYXJlZC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5hcHBseURGU09uQ29tcGxleGVzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgICAgLy8gTEdyYXBoPigpO1xyXG4gICAgICAgdmFyIG51bU9mTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBudW1PZk5vZGVzOyBpKyspXHJcbiAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBub2RlID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG4gICAgICAgICAgIFxyXG4gICAgICAgICAgICAvLyBUT0RPOiBJbnN0YW5jZSBvZiFcclxuICAgICAgICAgICAgaWYgKG5vZGUuaXNDb21wbGV4KCkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIC8vIGNvbXBsZXggaXMgZm91bmQsIHJlY3Vyc2Ugb24gaXQgdW50aWwgbm8gdmlzaXRlZCBjb21wbGV4IHJlbWFpbnMuXHJcbiAgICAgICAgICAgIGlmICghbm9kZS52aXNpdGVkKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB0aGlzLkRGU1Zpc2l0Q29tcGxleChub2RlKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgfVxyXG5cclxuICAgICAgIC8vIGNsZWFyIGVhY2ggY29tcGxleFxyXG4gICAgICAgdmFyIG51bU9mQ29tcGxleE9yZGVyID0gdGhpcy5jb21wbGV4T3JkZXIubGVuZ3RoO1xyXG4gICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBudW1PZkNvbXBsZXhPcmRlcjsgaSsrKVxyXG4gICAgICAge1xyXG4gICAgICAgICAgIGNsZWFyQ29tcGxleCh0aGlzLmNvbXBsZXhPcmRlcltpXSk7XHJcbiAgICAgICB9XHJcblxyXG4gICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS51cGRhdGVCb3VuZHMoKTtcclxuXHJcbiAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbigpO1xyXG4gICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5yZXNldEFsbEVkZ2VzKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCByZWN1cnNlcyBvbiB0aGUgY29tcGxleCBvYmplY3RzLiBJZiBhIG5vZGUgZG9lcyBub3QgY29udGFpblxyXG4qIGFueSBjb21wbGV4IG5vZGVzIG9yIGFsbCB0aGUgbm9kZXMgaW4gdGhlIGNoaWxkIGdyYXBoIGlzIGFscmVhZHkgbWFya2VkLFxyXG4qIGl0IGlzIHJlcG9ydGVkLiAoRGVwdGggZmlyc3QpXHJcbiogXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuREZTVmlzaXRDb21wbGV4ID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIG5vZGUpXHJcbntcclxuICAgIGlmIChub2RlLmdldENoaWxkKCkgIT0gbnVsbClcclxuICAgIHtcclxuICAgICAgICAgdmFyIG51bU9mQ2hpbGRyZW4gPSAgbm9kZS5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IG51bU9mQ2hpbGRyZW47IGkrKylcclxuICAgICAgICAge1xyXG4gICAgICAgICAgICAgdGhpcy5ERlNWaXNpdENvbXBsZXgobm9kZS5nZXRDaGlsZCgpLmdldE5vZGVzKClbaV0pO1xyXG4gICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKG5vZGUuaXNDb21wbGV4KCkgJiYgIW5vZGUuY29udGFpbnNVbm1hcmtlZENvbXBsZXgoKSlcclxuICAgIHtcclxuICAgICAgICAgdGhpcy5jb21wbGV4T3JkZXIucHVzaChub2RlKTtcclxuICAgICAgICAgbm9kZS52aXNpdGVkID0gdHJ1ZTtcclxuICAgICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgdGlsZXMgdGhlIGdpdmVuIGxpc3Qgb2Ygbm9kZXMgYnkgdXNpbmcgcG9seW9taW5vIHBhY2tpbmdcclxuKiBhbGdvcml0aG0uXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuYXBwbHlQb2x5b21pbm8gPSBmdW5jdGlvbiAoLypTYmduUEROb2RlKi8gcGFyZW50KVxyXG57XHJcbiAgICB2YXIgcmVjdDtcclxuICAgIHZhciBjaGlsZEdyID0gcGFyZW50LmdldENoaWxkKCk7XHJcblxyXG4gICAgaWYgKGNoaWxkR3IgPT0gbnVsbClcclxuICAgIHtcclxuICAgICAgICBjb25zb2xlLmxvZyhcIkNoaWxkIGdyYXBoIGlzIGVtcHR5IChQb2x5b21pbm8pXCIpO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIC8vIHBhY2tpbmcgdGFrZXMgdGhlIGlucHV0IGFzIGFuIGFycmF5LiBwdXQgdGhlIG1lbWJlcnMgaW4gYW4gYXJyYXkuXHJcbiAgICAgICAgdmFyIG1wQXJyYXkgPSBbXTtcclxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IGNoaWxkR3IuZ2V0Tm9kZXMoKS5sZW5ndGg7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG1wQXJyYXlbaV0gPSBjaGlsZEdyLmdldE5vZGVzKClbaV07XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBwYWNrIHJlY3RhbmdsZXNcclxuICAgICAgICBSZWN0UHJvYy5wYWNrUmVjdGFuZ2xlc01pbm8oXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUixcclxuICAgICAgICAgICAgICAgICAgICAgICAgbXBBcnJheS5sZW5ndGgsIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICBtcEFycmF5KTtcclxuXHJcbiAgICAgICAgLy8gYXBwbHkgY29tcGFjdGlvblxyXG4gICAgICAgIHZhciBjID0gbmV3IENvbXBhY3Rpb24oY2hpbGRHci5nZXROb2RlcygpKTtcclxuICAgICAgICBjLnBlcmZvcm0oKTtcclxuXHJcbiAgICAgICAgLy8gZ2V0IHRoZSByZXN1bHRpbmcgcmVjdGFuZ2xlIGFuZCBzZXQgcGFyZW50J3MgKGNvbXBsZXgpIHdpZHRoICZcclxuICAgICAgICAvLyBoZWlnaHRcclxuICAgICAgICByZWN0ID0gdGhpcy5jYWxjdWxhdGVCb3VuZHModHJ1ZSwgY2hpbGRHci5nZXROb2RlcygpKTtcclxuXHJcbiAgICAgICAgcGFyZW50LnNldFdpZHRoKHJlY3QuZ2V0V2lkdGgoKSk7XHJcbiAgICAgICAgcGFyZW50LnNldEhlaWdodChyZWN0LmdldEhlaWdodCgpKTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFJlYXNzaWducyB0aGUgY29tcGxleCBjb250ZW50LiBUaGUgb3V0ZXJtb3N0IGNvbXBsZXggaXMgcGxhY2VkIGZpcnN0LlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLnJlcG9wdWxhdGVDb21wbGV4ZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgZW1wdGllZER1bW15Q29tcGxleE1hcFNpemUgPSB0aGlzLmVtcHRpZWREdW1teUNvbXBsZXhNYXAua2V5U2V0KCkubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBlbXB0aWVkRHVtbXlDb21wbGV4TWFwU2l6ZTsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBjb21wID0gdGhpcy5lbXB0aWVkRHVtbXlDb21wbGV4TWFwLmtleVNldCgpW2ldO1xyXG4gICAgICAgIHZhciBjaEdyID0gdGhpcy5lbXB0aWVkRHVtbXlDb21wbGV4TWFwLmdldChjb21wKTtcclxuICAgICAgICBjb21wLnNldENoaWxkKGNoR3IpO1xyXG4gICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkucHVzaChjaEdyKTtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKHZhciBpID0gdGhpcy5jb21wbGV4T3JkZXIubGVuZ3RoIC0gMTsgaSA+PSAwOyBpLS0pXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGNvbXAgPSB0aGlzLmNvbXBsZXhPcmRlcltpXTtcclxuICAgICAgICB2YXIgY2hHciA9IHRoaXMuY2hpbGRHcmFwaE1hcC5nZXQoY29tcCk7XHJcblxyXG4gICAgICAgIC8vIHJlcG9wdWxhdGUgdGhlIGNvbXBsZXhcclxuICAgICAgICBjb21wLnNldENoaWxkKGNoR3IpO1xyXG5cclxuICAgICAgICAvLyBpZiB0aGUgY2hpbGQgZ3JhcGggaXMgbm90IG51bGwsIGFkanVzdCB0aGUgcG9zaXRpb25zIG9mIG1lbWJlcnNcclxuICAgICAgICBpZiAoY2hHciAhPSBudWxsKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgLy8gYWRqdXN0IHRoZSBwb3NpdGlvbnMgb2YgdGhlIG1lbWJlcnNcclxuICAgICAgICAgICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5QT0xZT01JTk9fUEFDS0lORylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5hZGp1c3RMb2NhdGlvbihjb21wLCBjaEdyKTtcclxuICAgICAgICAgICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkucHVzaChjaEdyKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlIGlmIChjb21wYWN0aW9uTWV0aG9kID09PSBTYmduUERMYXlvdXQuRGVmYXVsdENvbXBhY3Rpb25BbGdvcml0aG1FbnVtLlRJTElORylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5wdXNoKGNoR3IpO1xyXG5cclxuICAgICAgICAgICAgICAgIHZhciBwYWNrID0gdGhpcy5tZW1iZXJQYWNrTWFwLmdldChjb21wKTtcclxuICAgICAgICAgICAgICAgIHBhY2suYWRqdXN0TG9jYXRpb25zKGNvbXAuZ2V0TGVmdCgpLCBjb21wLmdldFRvcCgpKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIFxyXG4gICAgdmFyIGVtcHRpZWREdW1teUNvbXBsZXhNYXBTaXplID0gdGhpcy5lbXB0aWVkRHVtbXlDb21wbGV4TWFwLmtleVNldCgpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZW1wdGllZER1bW15Q29tcGxleE1hcFNpemU7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgY29tcCA9IHRoaXMuZW1wdGllZER1bW15Q29tcGxleE1hcC5rZXlTZXQoKVtpXTtcclxuICAgICAgICB2YXIgY2hHciA9IHRoaXMuZW1wdGllZER1bW15Q29tcGxleE1hcC5nZXQoY29tcCk7XHJcblxyXG4gICAgICAgIHRoaXMuYWRqdXN0TG9jYXRpb24oY29tcCwgY2hHcik7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5yZW1vdmVEdW1teUNvbXBsZXhlcygpO1xyXG5cclxuICAgIC8vIHJlc2V0XHJcbiAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbigpO1xyXG4gICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5yZXNldEFsbEVkZ2VzKCk7XHJcbiAgICB0aGlzLmNhbGN1bGF0ZU5vZGVzVG9BcHBseUdyYXZpdGF0aW9uVG8oKTtcclxufTtcclxuXHJcbi8qKlxyXG4qIEFkanVzdCBsb2NhdGlvbnMgb2YgY2hpbGRyZW4gb2YgZ2l2ZW4gY29tcGxleCB3cnQuIHRoZSBsb2NhdGlvbiBvZiB0aGVcclxuKiBjb21wbGV4XHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuYWRqdXN0TG9jYXRpb24gPSBmdW5jdGlvbiAoY29tcCwgY2hHcilcclxue1xyXG4gICAgdmFyIHJlY3QgPSBjYWxjdWxhdGVCb3VuZHMoZmFsc2UsIGNoR3IuZ2V0Tm9kZXMoKSk7XHJcblxyXG4gICAgdmFyIGRpZmZlcmVuY2VYID0gKHJlY3QueCAtIGNvbXAuZ2V0TGVmdCgpKTtcclxuICAgIHZhciBkaWZmZXJlbmNlWSA9IChyZWN0LnkgLSBjb21wLmdldFRvcCgpKTtcclxuXHJcbiAgICAvLyBpZiB0aGUgcGFyZW50IGdyYXBoIGlzIGEgY29tcG91bmQsIGFkZCBjb21wb3VuZCBtYXJnaW5zXHJcbiAgICBpZiAoY29tcC50eXBlICE9PSBTYmduUERDb25zdGFudHMuQ09NUExFWClcclxuICAgIHtcclxuICAgICAgICBkaWZmZXJlbmNlWCAtPSBTYmduUERDb25zdGFudHMuQ09NUE9VTkRfTk9ERV9NQVJHSU47XHJcbiAgICAgICAgZGlmZmVyZW5jZVkgLT0gU2JnblBEQ29uc3RhbnRzLkNPTVBPVU5EX05PREVfTUFSR0lOO1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIGogPSAwOyBqIDwgY2hHci5nZXROb2RlcygpLmxlbmd0aDsgaisrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzID0gY2hHci5nZXROb2Rlc1tqXTtcclxuXHJcbiAgICAgICAgcy5zZXRMb2NhdGlvbihzLmdldExlZnQoKSAtIGRpZmZlcmVuY2VYXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX0hPUklaT05UQUxfQlVGRkVSLCBzLmdldFRvcCgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC0gZGlmZmVyZW5jZVkgKyBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fVkVSVElDQUxfQlVGRkVSKTtcclxuXHJcbiAgICAgICAgaWYgKHMuZ2V0Q2hpbGQoKSAhPT0gbnVsbClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuYWRqdXN0TG9jYXRpb24ocywgcy5nZXRDaGlsZCgpKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBSZWN1cnNpdmVseSByZW1vdmVzIGFsbCBkdW1teSBjb21wbGV4IG5vZGVzIChwcmV2aW91c2x5IGNyZWF0ZWQgdG8gdGlsZVxyXG4qIGdyb3VwIGRlZ3JlZS16ZXJvIG5vZGVzKSBmcm9tIHRoZSBncmFwaC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jbGVhckR1bW15Q29tcGxleEdyYXBocyA9IGZ1bmN0aW9uIChjb21wKVxyXG57XHJcbiAgICBpZiAoY29tcC5nZXRDaGlsZCgpID09IG51bGwgfHwgY29tcC5pc0R1bW15Q29tcG91bmQpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IGNvbXAuZ2V0Q2hpbGQoKS5nZXROb2RlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbnVtT2ZDaGlsZHJlbjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBjaGlsZE5vZGUgPSBjb21wLmdldENoaWxkKCkuZ2V0Tm9kZXMoKVtpXTtcclxuICAgICAgICBpZiAoY2hpbGROb2RlLmdldENoaWxkKCkgIT0gbnVsbCAmJiBjaGlsZE5vZGUuZ2V0RWRnZXMoKS5sZW5ndGggPT0gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuY2xlYXJEdW1teUNvbXBsZXhHcmFwaHMoY2hpbGROb2RlKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5ncmFwaE1hbmFnZXIuZ2V0R3JhcGhzKCkuaW5jbHVkZXMoY29tcC5nZXRDaGlsZCgpKSlcclxuICAgIHtcclxuICAgICAgICBpZiAodGhpcy5jYWxjR3JhcGhEZWdyZWUoY29tcCkgPT09IDApXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLmVtcHRpZWREdW1teUNvbXBsZXhNYXAucHV0KGNvbXAsIGNvbXAuZ2V0Q2hpbGQoKSk7XHJcblxyXG4gICAgICAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLmdldEdyYXBocygpLnNwbGljZShcclxuICAgICAgICAgICAgICAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLmdldEdyYXBocygpLmluZGV4T2YoY29tcC5nZXRDaGlsZCgpKSwgMSk7XHJcbiAgICAgICAgICAgIGNvbXAuc2V0Q2hpbGQobnVsbCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogRHVtbXkgY29tcGxleGVzIChwbGFjZWQgaW4gdGhlIFwiZHVtbXlDb21wbGV4TGlzdFwiKSBhcmUgcmVtb3ZlZCBmcm9tIHRoZVxyXG4qIGdyYXBoLlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLnJlbW92ZUR1bW15Q29tcGxleGVzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGR1bW15Q29tcGxleExpc3RTaXplID0gdGhpcy5kdW1teUNvbXBsZXhMaXN0Lmxlbmd0aDtcclxuICAgIC8vIHJlbW92ZSBkdW1teSBjb21wbGV4ZXMgYW5kIGNvbm5lY3QgY2hpbGRyZW4gdG8gb3JpZ2luYWwgcGFyZW50XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8ZHVtbXlDb21wbGV4TGlzdFNpemU7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZHVtbXlDb21wbGV4ID0gdGhpcy5kdW1teUNvbXBsZXhMaXN0W2ldO1xyXG4gICAgICAgIHZhciBjaGlsZEdyYXBoID0gZHVtbXlDb21wbGV4LmdldENoaWxkKCk7XHJcbiAgICAgICAgdmFyIG93bmVyID0gZHVtbXlDb21wbGV4LmdldE93bmVyKCk7XHJcblxyXG4gICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkuc3BsaWNlKFxyXG4gICAgICAgICAgICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5pbmRleE9mKGNoaWxkR3JhcGgpLCAxKTtcclxuICAgICAgICBkdW1teUNvbXBsZXguc2V0Q2hpbGQobnVsbCk7XHJcblxyXG4gICAgICAgIG93bmVyLnJlbW92ZShkdW1teUNvbXBsZXgpO1xyXG5cclxuICAgICAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IGNoaWxkR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgaj0wOyBqPG51bU9mQ2hpbGRyZW47IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG93bmVyLmFkZChjaGlsZEdyYXBoLmdldE5vZGVzKClbal0pO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIHJldHVybnMgdGhlIGJvdW5kaW5nIHJlY3RhbmdsZSBvZiB0aGUgZ2l2ZW4gc2V0IG9mIG5vZGVzIHdpdGhcclxuKiBvciB3aXRob3V0IHRoZSBtYXJnaW5zXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY3VsYXRlQm91bmRzID0gZnVuY3Rpb24gKGlzTWFyZ2luSW5jbHVkZWQsIG5vZGVzKVxyXG57XHJcbiAgICB2YXIgYm91bmRMZWZ0ID0gSW50ZWdlci5NQVhfVkFMVUU7XHJcbiAgICB2YXIgYm91bmRSaWdodCA9IEludGVnZXIuTUlOX1ZBTFVFO1xyXG4gICAgdmFyIGJvdW5kVG9wID0gSW50ZWdlci5NQVhfVkFMVUU7XHJcbiAgICB2YXIgYm91bmRCb3R0b20gPSBJbnRlZ2VyLk1JTl9WQUxVRTtcclxuICAgIHZhciBub2RlTGVmdDtcclxuICAgIHZhciBub2RlUmlnaHQ7XHJcbiAgICB2YXIgbm9kZVRvcDtcclxuICAgIHZhciBub2RlQm90dG9tO1xyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IG5vZGVzLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNoaWxkcmVuOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGxOb2RlID0gbm9kZXNbaV07XHJcbiAgICAgICAgbm9kZUxlZnQgPSBsTm9kZS5nZXRMZWZ0KCk7XHJcbiAgICAgICAgbm9kZVJpZ2h0ID0gbE5vZGUuZ2V0UmlnaHQoKTtcclxuICAgICAgICBub2RlVG9wID0gbE5vZGUuZ2V0VG9wKCk7XHJcbiAgICAgICAgbm9kZUJvdHRvbSA9IGxOb2RlLmdldEJvdHRvbSgpO1xyXG5cclxuICAgICAgICBpZiAoYm91bmRMZWZ0ID4gbm9kZUxlZnQpXHJcbiAgICAgICAgICAgIGJvdW5kTGVmdCA9IG5vZGVMZWZ0O1xyXG5cclxuICAgICAgICBpZiAoYm91bmRSaWdodCA8IG5vZGVSaWdodClcclxuICAgICAgICAgICAgYm91bmRSaWdodCA9IG5vZGVSaWdodDtcclxuXHJcbiAgICAgICAgaWYgKGJvdW5kVG9wID4gbm9kZVRvcClcclxuICAgICAgICAgICAgYm91bmRUb3AgPSBub2RlVG9wO1xyXG5cclxuICAgICAgICBpZiAoYm91bmRCb3R0b20gPCBub2RlQm90dG9tKVxyXG4gICAgICAgICAgICBib3VuZEJvdHRvbSA9IG5vZGVCb3R0b207XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGlzTWFyZ2luSW5jbHVkZWQpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIG5ldyBSZWN0YW5nbGVEKGJvdW5kTGVmdCAtIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4sIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICBib3VuZFRvcCAtIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4sIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICBib3VuZFJpZ2h0IC0gYm91bmRMZWZ0ICsgMiAqIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4sXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIGJvdW5kQm90dG9tIC0gYm91bmRUb3AgKyAyICogU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTik7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIG5ldyBSZWN0YW5nbGVEKGJvdW5kTGVmdCwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIGJvdW5kVG9wLCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgYm91bmRSaWdodCAtIGJvdW5kTGVmdCwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIGJvdW5kQm90dG9tIC0gYm91bmRUb3ApO1xyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogY2FsY3VsYXRlcyB1c2VkQXJlYS90b3RhbEFyZWEgaW5zaWRlIHRoZSBjb21wbGV4ZXMgYW5kIHByaW50cyB0aGVtIG91dC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jYWxjdWxhdGVGdWxsbmVzc09mQ29tcGxleGVzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGxhcmdlc3RDb21wbGV4ID0gbnVsbDtcclxuICAgIHZhciB0b3RhbEFyZWEgPSAwLjA7XHJcbiAgICB2YXIgdXNlZEFyZWEgPSAwLjA7XHJcbiAgICB2YXIgbWF4QXJlYSA9IE51bWJlci5NSU5fVkFMVUU7XHJcblxyXG4gICAgLy8gZmluZCB0aGUgbGFyZ2VzdCBjb21wbGV4IC0+IGFyZWFcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy5nZXRBbGxOb2RlcygpLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG4gICAgICAgIGlmICgocy50eXBlID09PSBTYmduUERDb25zdGFudHMuQ09NUExFWCkgJiYgXHJcbiAgICAgICAgICAgICgocy5nZXRXaWR0aCgpICogcy5nZXRIZWlnaHQoKSkgPiBtYXhBcmVhKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG1heEFyZWEgPSBzLmdldFdpZHRoKCkgKiBzLmdldEhlaWdodCgpO1xyXG4gICAgICAgICAgICBsYXJnZXN0Q29tcGxleCA9IHM7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHVzZWRBcmVhID0gdGhpcy5jYWxjdWxhdGVVc2VkQXJlYShsYXJnZXN0Q29tcGxleCk7XHJcbiAgICB0b3RhbEFyZWEgPSBsYXJnZXN0Q29tcGxleC5nZXRXaWR0aCgpICogbGFyZ2VzdENvbXBsZXguZ2V0SGVpZ2h0KCk7XHJcblxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5USUxJTkcpXHJcbiAgICAgICAgICAgIGNvbnNvbGUubG9nKFwiVGlsaW5nIHJlc3VsdHNcIik7XHJcbiAgICBlbHNlIGlmICh0aGlzLmNvbXBhY3Rpb25NZXRob2QgPT09IFNiZ25QRExheW91dC5EZWZhdWx0Q29tcGFjdGlvbkFsZ29yaXRobUVudW0uUE9MWU9NSU5PX1BBQ0tJTkcpXHJcbiAgICAgICAgICAgIGNvbnNvbGUubG9nKFwiUG9seW9taW5vIFBhY2tpbmcgcmVzdWx0c1wiKTtcclxuXHJcbiAgICBjb25zb2xlLmxvZyhcIiA9IFwiICsgdXNlZEFyZWEgLyB0b3RhbEFyZWEpO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY2FsY3VsYXRlcyB0aGUgdXNlZCBhcmVhIG9mIGEgZ2l2ZW4gY29tcGxleCBub2RlJ3MgY2hpbGRyZW5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jYWxjdWxhdGVVc2VkQXJlYSA9IGZ1bmN0aW9uIChwYXJlbnQpXHJcbntcclxuICAgIHZhciB0b3RhbEFyZWEgPSAwO1xyXG4gICAgaWYgKHBhcmVudC5nZXRDaGlsZCgpID09IG51bGwpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIDAuMDtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHBhcmVudC5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIG5vZGUgPSBwYXJlbnQuZ2V0Q2hpbGQoKS5nZXROb2RlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAobm9kZS50eXBlICE9PSBTYmduUERDb25zdGFudHMuQ09NUExFWClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRvdGFsQXJlYSArPSBub2RlLmdldFdpZHRoKCkgKiBub2RlLmdldEhlaWdodCgpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0b3RhbEFyZWEgKz0gdGhpcy5jYWxjdWxhdGVVc2VkQXJlYShub2RlKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBcclxuICAgIHJldHVybiB0b3RhbEFyZWE7XHJcbn07XHJcblxyXG4vLyAqKioqKioqKioqKioqKioqKioqKiogU0VDVElPTiA6IE9WRVJSSURFTiBNRVRIT0RTICoqKioqKioqKioqKioqKioqKioqKlxyXG5cclxuLyoqXHJcbiAqIFRoaXMgbWV0aG9kIGNyZWF0ZXMgYSBuZXcgbm9kZSBhc3NvY2lhdGVkIHdpdGggdGhlIGlucHV0IHZpZXcgbm9kZS5cclxuICovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUubmV3Tm9kZSA9IGZ1bmN0aW9uICh2Tm9kZSlcclxue1xyXG4gICAgcmV0dXJuIG5ldyBTYmduUEROb2RlKHRoaXMuZ3JhcGhNYW5hZ2VyLCBudWxsLCBudWxsLCB2Tm9kZSwgbnVsbCk7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgY3JlYXRlcyBhIG5ldyBlZGdlIGFzc29jaWF0ZWQgd2l0aCB0aGUgaW5wdXQgdmlldyBlZGdlLlxyXG4gKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5uZXdFZGdlID0gZnVuY3Rpb24gKHZFZGdlKVxyXG57XHJcbiAgICByZXR1cm4gbmV3IFNiZ25QREVkZ2UobnVsbCwgbnVsbCwgdkVkZ2UpO1xyXG59O1xyXG5cclxuLyoqXHJcbiAqIFRoaXMgbWV0aG9kIHBlcmZvcm1zIGxheW91dCBvbiBjb25zdHJ1Y3RlZCBsLWxldmVsIGdyYXBoLiBJdCByZXR1cm5zIHRydWVcclxuICogb24gc3VjY2VzcywgZmFsc2Ugb3RoZXJ3aXNlLlxyXG4gKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5sYXlvdXQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgYiA9IGZhbHNlO1xyXG5cclxuICAgIHRoaXMuZ3JvdXBaZXJvRGVncmVlTWVtYmVycygpO1xyXG4gICAgdGhpcy5hcHBseURGU09uQ29tcGxleGVzKCk7XHJcbiAgICBiID0gQ29TRUxheW91dC5wcm90b3R5cGUubGF5b3V0LmNhbGwodGhpcywgYXJndW1lbnRzKTtcclxuICAgIHRoaXMucmVwb3B1bGF0ZUNvbXBsZXhlcygpO1xyXG5cclxuICAgIHRoaXMuZ2V0QWxsTm9kZXMoKTtcclxuICAgIHJldHVybiBiO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgdXNlcyBjbGFzc2ljIGxheW91dCBtZXRob2QgKHdpdGhvdXQgbXVsdGktc2NhbGluZylcclxuKiBNb2RpZmljYXRpb246IGNyZWF0ZSBwb3J0IG5vZGVzIGFmdGVyIHJhbmRvbSBwb3NpdGlvbmluZ1xyXG4qL1xyXG4vL0BPdmVycmlkZVxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmNsYXNzaWNMYXlvdXQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB0aGlzLmNhbGN1bGF0ZU5vZGVzVG9BcHBseUdyYXZpdGF0aW9uVG8oKTtcclxuXHJcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5jYWxjTG93ZXN0Q29tbW9uQW5jZXN0b3JzKCk7XHJcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5jYWxjSW5jbHVzaW9uVHJlZURlcHRocygpO1xyXG5cclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLmdldFJvb3QoKS5jYWxjRXN0aW1hdGVkU2l6ZSgpO1xyXG4gICAgdGhpcy5jYWxjSWRlYWxFZGdlTGVuZ3RocygpO1xyXG5cclxuICAgIGlmICghdGhpcy5pbmNyZW1lbnRhbClcclxuICAgIHtcclxuICAgICAgICB2YXIgZm9yZXN0ID0gdGhpcy5nZXRGbGF0Rm9yZXN0KCk7XHJcblxyXG4gICAgICAgIGlmIChmb3Jlc3QubGVuZ3RoID4gMClcclxuICAgICAgICAvLyBUaGUgZ3JhcGggYXNzb2NpYXRlZCB3aXRoIHRoaXMgbGF5b3V0IGlzIGZsYXQgYW5kIGEgZm9yZXN0XHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLnBvc2l0aW9uTm9kZXNSYWRpYWxseShmb3Jlc3QpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAgLy8gVGhlIGdyYXBoIGFzc29jaWF0ZWQgd2l0aCB0aGlzIGxheW91dCBpcyBub3QgZmxhdCBvciBhIGZvcmVzdFxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGhpcy5wb3NpdGlvbk5vZGVzUmFuZG9tbHkoKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKCF0aGlzLmFyZVBvcnROb2Rlc0NyZWF0ZWQoKSlcclxuICAgIHtcclxuICAgICAgICB0aGlzLmNyZWF0ZVBvcnROb2RlcygpO1xyXG4gICAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgICAgICB0aGlzLmdyYXBoTWFuYWdlci5yZXNldEFsbE5vZGVzVG9BcHBseUdyYXZpdGF0aW9uKCk7XHJcbiAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxFZGdlcygpO1xyXG4gICAgICAgIHRoaXMuY2FsY3VsYXRlTm9kZXNUb0FwcGx5R3Jhdml0YXRpb25UbygpO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB0aGlzLmluaXRTcHJpbmdFbWJlZGRlcigpO1xyXG4gICAgdGhpcy5ydW5TcHJpbmdFbWJlZGRlcigpO1xyXG5cclxuICAgIHJldHVybiB0cnVlO1xyXG59O1xyXG5cclxuXHJcbi8qKlxyXG4gKiBUaGlzIG1ldGhvZCBjYWxjdWxhdGVzIHRoZSBzcHJpbmcgZm9yY2VzIGZvciB0aGUgZW5kcyBvZiBlYWNoIG5vZGUuXHJcbiAqIE1vZGlmaWNhdGlvbjogZG8gbm90IGNhbGN1bGF0ZSBzcHJpbmcgZm9yY2UgZm9yIHJpZ2lkIGVkZ2VzXHJcbiAqL1xyXG4vL0BPdmVycmlkZVxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmNhbGNTcHJpbmdGb3JjZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgbEVkZ2VzID0gdGhpcy5nZXRBbGxFZGdlcygpO1xyXG4gICAgdmFyIGVkZ2U7XHJcblxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsRWRnZXMubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgZWRnZSA9IGxFZGdlc1tpXTtcclxuXHJcbiAgICAgICAgaWYgKGVkZ2UudHlwZSAhPT0gU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0UpXHJcbiAgICAgICAgeyAgICBcclxuICAgICAgICAgICAgdGhpcy5jYWxjU3ByaW5nRm9yY2UoZWRnZSwgZWRnZS5pZGVhbExlbmd0aCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHQgXHJcbiAqIFRoaXMgbWV0aG9kIGNhbGN1bGF0ZXMgdGhlIHJlcHVsc2lvbiBmb3JjZXMgZm9yIGVhY2ggcGFpciBvZiBub2Rlcy5cclxuICogTW9kaWZpY2F0aW9uOiBEbyBub3QgY2FsY3VsYXRlIHJlcHVsc2lvbiBmb3IgcG9ydCAmIHByb2Nlc3Mgbm9kZXNcclxuICovXHJcbi8vQE92ZXJyaWRlXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY1JlcHVsc2lvbkZvcmNlcyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBpLCBqO1xyXG4gICAgdmFyIG5vZGVBLCBub2RlQjtcclxuICAgIHZhciBsTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCk7XHJcbiAgICB2YXIgcHJvY2Vzc2VkTm9kZVNldDtcclxuICAgIFxyXG4gICAgaWYgKHRoaXMudXNlRlJHcmlkVmFyaWFudClcclxuICAgIHtcclxuICAgICAgICAvLyBncmlkIGlzIGEgdmVjdG9yIG1hdHJpeCB0aGF0IGhvbGRzIENvU0VOb2Rlcy5cclxuICAgICAgICAvLyBiZSBzdXJlIHRvIGNvbnZlcnQgdGhlIE9iamVjdCB0eXBlIHRvIENvU0VOb2RlLlxyXG4gICAgICAgIGlmICh0aGlzLnRvdGFsSXRlcmF0aW9uc1xyXG4gICAgICAgICAgICAgICAgJSBTYmduUERDb25zdGFudHMuR1JJRF9DQUxDVUxBVElPTl9DSEVDS19QRVJJT0QgPT0gMSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuZ3JpZCA9IHRoaXMuY2FsY0dyaWQodGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpKTtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIC8vIHB1dCBhbGwgbm9kZXMgdG8gcHJvcGVyIGdyaWQgY2VsbHNcclxuICAgICAgICAgICAgZm9yIChpID0gMDsgaSA8IGxOb2Rlcy5sZW5ndGg7IGkrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgbm9kZUEgPSBsTm9kZXNbaV07XHJcbiAgICAgICAgICAgICAgICB0aGlzLmFkZE5vZGVUb0dyaWQobm9kZUEsIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuZ3JpZCwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldExlZnQoKSwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldFRvcCgpKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgcHJvY2Vzc2VkTm9kZVNldCA9IG5ldyBIYXNoU2V0KCk7XHJcblxyXG4gICAgICAgIC8vIGNhbGN1bGF0ZSByZXB1bHNpb24gZm9yY2VzIGJldHdlZW4gZWFjaCBub2RlcyBhbmQgaXRzIHN1cnJvdW5kaW5nXHJcbiAgICAgICAgZm9yIChpID0gMDsgaSA8IGxOb2Rlcy5sZW5ndGg7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG5vZGVBID0gbE5vZGVzW2ldO1xyXG4gICAgICAgICAgICB0aGlzLmNhbGN1bGF0ZVJlcHVsc2lvbkZvcmNlT2ZBTm9kZSh0aGlzLmdyaWQsIG5vZGVBLCBwcm9jZXNzZWROb2RlU2V0KTtcclxuICAgICAgICAgICAgcHJvY2Vzc2VkTm9kZVNldC5hZGQobm9kZUEpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICBmb3IgKGkgPSAwOyBpIDwgbE5vZGVzLmxlbmd0aDsgaSsrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgbm9kZUEgPSBsTm9kZXNbaV07XHJcblxyXG4gICAgICAgICAgICBmb3IgKGogPSBpICsgMTsgaiA8IGxOb2Rlcy5sZW5ndGg7IGorKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgbm9kZUIgPSBsTm9kZXNbal07XHJcblxyXG4gICAgICAgICAgICAgICAgLy8gSWYgYm90aCBub2RlcyBhcmUgbm90IG1lbWJlcnMgb2YgdGhlIHNhbWUgZ3JhcGgsIHNraXAuXHJcbiAgICAgICAgICAgICAgICBpZiAobm9kZUEuZ2V0T3duZXIoKSAhPT0gbm9kZUIuZ2V0T3duZXIoKSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgICAgICBpZiAobm9kZUEudHlwZSAhPT0gbnVsbCAmJiBcclxuICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlICE9PSBudWxsICYmIFxyXG4gICAgICAgICAgICAgICAgICAgIG5vZGVBLmdldE93bmVyKCkgPT09IG5vZGVCLmdldE93bmVyKCkgJiYgXHJcbiAgICAgICAgICAgICAgICAgICAgKG5vZGVBLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICBub2RlQS50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQgfHwgXHJcbiAgICAgICAgICAgICAgICAgICAgIG5vZGVCLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgICAgIHRoaXMuY2FsY1JlcHVsc2lvbkZvcmNlKG5vZGVBLCBub2RlQik7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgZmluZHMgc3Vycm91bmRpbmcgbm9kZXMgb2Ygbm9kZUEgaW4gcmVwdWxzaW9uIHJhbmdlLlxyXG4gKiBBbmQgY2FsY3VsYXRlcyB0aGUgcmVwdWxzaW9uIGZvcmNlcyBiZXR3ZWVuIG5vZGVBIGFuZCBpdHMgc3Vycm91bmRpbmcuXHJcbiAqIER1cmluZyB0aGUgY2FsY3VsYXRpb24sIGlnbm9yZXMgdGhlIG5vZGVzIHRoYXQgaGF2ZSBhbHJlYWR5IGJlZW4gcHJvY2Vzc2VkLlxyXG4gKiBNb2RpZmljYXRpb246IERvIG5vdCBjYWxjdWxhdGUgcmVwdWxzaW9uIGZvciBwb3J0ICYgcHJvY2VzcyBub2Rlc1xyXG4gKi9cclxuLy8gQE92ZXJyaWRlXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY3VsYXRlUmVwdWxzaW9uRm9yY2VPZkFOb2RlID0gZnVuY3Rpb24gKGdyaWQsIG5vZGVBLCBwcm9jZXNzZWROb2RlU2V0KVxyXG57XHJcbiAgICB2YXIgaSwgajtcclxuXHJcbiAgICBpZiAodGhpcy50b3RhbEl0ZXJhdGlvbnMgJSBGRExheW91dENvbnN0YW50cy5HUklEX0NBTENVTEFUSU9OX0NIRUNLX1BFUklPRCA9PSAxKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzdXJyb3VuZGluZyA9IG5ldyBIYXNoU2V0KCk7XHJcbiAgICAgICAgdmFyIG5vZGVCO1xyXG5cclxuICAgICAgICBmb3IgKGkgPSAobm9kZUEuc3RhcnRYIC0gMSk7IGkgPCAobm9kZUEuZmluaXNoWCArIDIpOyBpKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBmb3IgKGogPSAobm9kZUEuc3RhcnRZIC0gMSk7IGogPCAobm9kZUEuZmluaXNoWSArIDIpOyBqKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmICghKChpIDwgMCkgfHwgKGogPCAwKSB8fCAoaSA+PSBncmlkLmxlbmd0aCkgfHwgKGogPj0gZ3JpZFswXS5sZW5ndGgpKSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICB2YXIgbnVtT2ZOb2RlcyA9IGdyaWRbaV1bal0ubGVuZ3RoO1xyXG4gICAgICAgICAgICAgICAgICAgIGZvciAodmFyIGsgPSAwOyBrIDwgbnVtT2ZOb2RlczsgaysrKVxyXG4gICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgbm9kZUIgPSBncmlkW2ldW2pdW2tdO1xyXG5cclxuICAgICAgICAgICAgICAgICAgICAgICAgLy8gSWYgYm90aCBub2RlcyBhcmUgbm90IG1lbWJlcnMgb2YgdGhlIHNhbWUgZ3JhcGgsXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC8vIG9yIGJvdGggbm9kZXMgYXJlIHRoZSBzYW1lLCBza2lwLlxyXG4gICAgICAgICAgICAgICAgICAgICAgICBpZiAoKG5vZGVBLmdldE93bmVyKCkgIT09IG5vZGVCLmdldE93bmVyKCkpIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKG5vZGVBID09PSBub2RlQikpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAgICAgICAgICAgICBpZiAobm9kZUEudHlwZSAhPT0gbnVsbCAmJiBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIG5vZGVCLnR5cGUgIT09IG51bGwgJiYgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlQS5nZXRPd25lcigpID09PSBub2RlQi5nZXRPd25lcigpICYmIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKG5vZGVBLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgIG5vZGVBLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVCB8fCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlID09PSBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVCB8fCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgICAgICAgICAgICAgLy8gY2hlY2sgaWYgdGhlIHJlcHVsc2lvbiBmb3JjZSBiZXR3ZWVuXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC8vIG5vZGVBIGFuZCBub2RlQiBoYXMgYWxyZWFkeSBiZWVuIGNhbGN1bGF0ZWRcclxuICAgICAgICAgICAgICAgICAgICAgICAgaWYgKCFwcm9jZXNzZWROb2RlU2V0LmNvbnRhaW5zKG5vZGVCKSAmJiAhc3Vycm91bmRpbmcuY29udGFpbnMobm9kZUIpKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICB2YXIgZGlzdGFuY2VYID0gTWF0aC5hYnMobm9kZUEuZ2V0Q2VudGVyWCgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgLSBub2RlQi5nZXRDZW50ZXJYKCkpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgLSAoKG5vZGVBLmdldFdpZHRoKCkgLyAyKSArIChub2RlQlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAuZ2V0V2lkdGgoKSAvIDIpKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHZhciBkaXN0YW5jZVkgPSBNYXRoLmFicyhub2RlQS5nZXRDZW50ZXJZKClcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAtIG5vZGVCLmdldENlbnRlclkoKSlcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAtICgobm9kZUEuZ2V0SGVpZ2h0KCkgLyAyKSArIChub2RlQlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAuZ2V0SGVpZ2h0KCkgLyAyKSk7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIC8vIGlmIHRoZSBkaXN0YW5jZSBiZXR3ZWVuIG5vZGVBIGFuZCBub2RlQlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgLy8gaXMgbGVzcyB0aGVuIGNhbGN1bGF0aW9uIHJhbmdlXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBpZiAoKGRpc3RhbmNlWCA8PSB0aGlzLnJlcHVsc2lvblJhbmdlKSAmJiBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAoZGlzdGFuY2VZIDw9IHRoaXMucmVwdWxzaW9uUmFuZ2UpKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIC8vIHRoZW4gYWRkIG5vZGVCIHRvIHN1cnJvdW5kaW5nIG9mIG5vZGVBXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgc3Vycm91bmRpbmcuYWRkKG5vZGVCKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICBcclxuICAgICAgICBub2RlQS5zdXJyb3VuZGluZyA9IHN1cnJvdW5kaW5nLnNldDtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKGkgPSAwOyBpIDwgbm9kZUEuc3Vycm91bmRpbmcubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5jYWxjUmVwdWxzaW9uRm9yY2Uobm9kZUEsIG5vZGVBLnN1cnJvdW5kaW5nW2ldKTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIGNyZWF0ZXMgYSBwb3J0IG5vZGUgd2l0aCB0aGUgYXNzb2NpYXRlZCB0eXBlIChpbnB1dC9vdXRwdXRcclxuKiBwb3J0KVxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLm5ld1BvcnROb2RlID0gZnVuY3Rpb24gKHZOb2RlLCB0eXBlKVxyXG57XHJcbiAgICB2YXIgbiA9IG5ldyBTYmduUEROb2RlKHRoaXMuZ3JhcGhNYW5hZ2VyLCBudWxsLCBudWxsLCB2Tm9kZSwgbnVsbCk7XHJcbiAgICBuLnR5cGUgPSB0eXBlO1xyXG4gICAgbi5zZXRXaWR0aChTYmduUERDb25zdGFudHMuUE9SVF9OT0RFX0RFRkFVTFRfV0lEVEgpO1xyXG4gICAgbi5zZXRIZWlnaHQoU2JnblBEQ29uc3RhbnRzLlBPUlRfTk9ERV9ERUZBVUxUX0hFSUdIVCk7XHJcblxyXG4gICAgcmV0dXJuIG47XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBjcmVhdGVzIGFuIFNCR05Qcm9jZXNzTm9kZSBvYmplY3RcclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5uZXdQcm9jZXNzTm9kZSA9IGZ1bmN0aW9uICh2Tm9kZSlcclxue1xyXG4gICAgcmV0dXJuIG5ldyBTYmduUHJvY2Vzc05vZGUodGhpcy5ncmFwaE1hbmFnZXIsIG51bGwsIG51bGwsIHZOb2RlKTtcclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIGNyZWF0ZXMgYSByaWdpZCBlZGdlLlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLm5ld1JpZ2lkRWRnZSA9IGZ1bmN0aW9uICh2RWRnZSlcclxue1xyXG4gICAgdmFyIGUgPSBuZXcgU2JnblBERWRnZShudWxsLCBudWxsLCB2RWRnZSk7XHJcbiAgICBlLnR5cGUgPSBTYmduUERDb25zdGFudHMuUklHSURfRURHRTtcclxuICAgIHJldHVybiBlO1xyXG59O1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBTYmduUERMYXlvdXQ7XHJcbiIsInZhciBDb1NFTm9kZSA9IHJlcXVpcmUoJy4vQ29TRU5vZGUnKTtcclxudmFyIFNiZ25QRENvbnN0YW50cyA9IHJlcXVpcmUoJy4vU2JnblBEQ29uc3RhbnRzJyk7XHJcbnZhciBQb2ludEQgPSByZXF1aXJlKCcuL1BvaW50RCcpO1xyXG5cclxuLy8gVE9ETzogVGhlcmUgaXMgYW5vdGhlciBjb250cnVjdG9yIGF2YWlsYWJsZSBpbiBqYXZhLCBkbyB3ZSBuZWVkIGl0P1xyXG5cclxuZnVuY3Rpb24gU2JnblBETm9kZShnbSwgbG9jLCBzaXplLCB2Tm9kZSwgdHlwZSkgXHJcbntcclxuICAgIENvU0VOb2RlLmNhbGwodGhpcywgZ20sIGxvYywgc2l6ZSwgdk5vZGUpO1xyXG4gICAgXHJcbiAgICB0aGlzLnR5cGUgPSB0eXBlO1xyXG4gICAgdGhpcy52aXNpdGVkID0gZmFsc2U7XHJcbiAgICBcclxuICAgIHRoaXMubGFiZWwgPSB2Tm9kZSA/IHZOb2RlLmxhYmVsIDogbnVsbDtcclxuICAgIHRoaXMuaXNEdW1teUNvbXBvdW5kID0gZmFsc2U7XHJcbn1cclxuXHJcblNiZ25QRE5vZGUucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShDb1NFTm9kZS5wcm90b3R5cGUpO1xyXG5mb3IgKHZhciBwcm9wIGluIENvU0VOb2RlKSB7XHJcbiAgU2JnblBETm9kZVtwcm9wXSA9IENvU0VOb2RlW3Byb3BdO1xyXG59XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5jb3B5ID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIG5vZGUpIFxyXG57XHJcbiAgICB0aGlzLnR5cGUgPSBub2RlLnR5cGU7XHJcbiAgICB0aGlzLmxhYmVsID0gbm9kZS5sYWJlbDtcclxuICAgIHRoaXMuc2V0Q2VudGVyKG5vZGUuZ2V0Q2VudGVyWCgpLCBub2RlLmdldENlbnRlclkoKSk7XHJcbiAgICB0aGlzLnNldENoaWxkKG5vZGUuZ2V0Q2hpbGQoKSk7XHJcbiAgICB0aGlzLnNldEhlaWdodChub2RlLmdldEhlaWdodCgpKTtcclxuICAgIHRoaXMuc2V0TG9jYXRpb24obm9kZS5nZXRMb2NhdGlvbigpLngsIG5vZGUuZ2V0TG9jYXRpb24oKS55KTtcclxuICAgIHRoaXMuc2V0TmV4dChub2RlLmdldE5leHQoKSk7XHJcbiAgICB0aGlzLnNldE93bmVyKG5vZGUuZ2V0T3duZXIoKSk7XHJcbiAgICB0aGlzLnNldFByZWQxKG5vZGUuZ2V0UHJlZDEoKSk7XHJcbiAgICB0aGlzLnNldFByZWQyKG5vZGUuZ2V0UHJlZDIoKSk7XHJcbiAgICB0aGlzLnNldFdpZHRoKG5vZGUuZ2V0V2lkdGgoKSk7XHJcbn07XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5nZXRTcHJpbmdGb3JjZVggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy5zcHJpbmdGb3JjZVg7XHJcbn07XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5pc0NvbXBsZXggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLkNPTVBMRVgpID09PSAwO1xyXG59O1xyXG5cclxuU2JnblBETm9kZS5wcm90b3R5cGUuaXNJbnB1dFBvcnQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLklOUFVUX1BPUlQpID09PSAwO1xyXG59O1xyXG5cclxuU2JnblBETm9kZS5wcm90b3R5cGUuaXNPdXRwdXRQb3J0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMudHlwZS5sb2NhbGVDb21wYXJlKFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVCkgPT09IDA7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgY2hlY2tzIGlmIHRoZSBnaXZlbiBub2RlIGNvbnRhaW5zIGFueSB1bm1hcmtlZCBjb21wbGV4IG5vZGVzXHJcbiAqIGluIGl0cyBjaGlsZCBncmFwaC5cclxuKi9cclxuU2JnblBETm9kZS5wcm90b3R5cGUuY29udGFpbnNVbm1hcmtlZENvbXBsZXggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICBpZiAodGhpcy5nZXRDaGlsZCgpID09IG51bGwpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIHZhciBudW1PZkNoaWxkcmVuID0gdGhpcy5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGluZGV4PTA7IGluZGV4PG51bU9mQ2hpbGRyZW47IGluZGV4KyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgY2hpbGQgPSB0aGlzLmdldENoaWxkKCkuZ2V0Tm9kZXMoKVtpbmRleF07XHJcbiAgICAgICAgICAgIGlmIChjaGlsZC5pc0NvbXBsZXgoKSAmJiAhY2hpbGQudmlzaXRlZClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblBETm9kZS5wcm90b3R5cGUucmVzZXRGb3JjZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB0aGlzLnNwcmluZ0ZvcmNlWCA9IDA7XHJcbiAgICB0aGlzLnNwcmluZ0ZvcmNlWSA9IDA7XHJcbiAgICB0aGlzLnJlcHVsc2lvbkZvcmNlWCA9IDA7XHJcbiAgICB0aGlzLnJlcHVsc2lvbkZvcmNlWSA9IDA7XHJcbn07XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5yb3RhdGVOb2RlID0gZnVuY3Rpb24gKC8qUG9pbnREKi8gb3JpZ2luLCAvKmludCovIHJvdGF0aW9uRGVncmVlKVxyXG57XHJcbiAgICB2YXIgcmVsYXRpdmVQdCA9IG5ldyBQb2ludEQoXHJcbiAgICAgICAgICAgICh0aGlzLmdldENlbnRlclgoKSAtIG9yaWdpbi54KSwgKHRoaXMuZ2V0Q2VudGVyWSgpIC0gb3JpZ2luLnkpKTtcclxuICAgIHZhciByb3RhdGVkUHQgPSBuZXcgUG9pbnREKFxyXG4gICAgICAgICAgICAoLU1hdGguc2lnbnVtKHJvdGF0aW9uRGVncmVlKSAqIHJlbGF0aXZlUHQueSksIFxyXG4gICAgICAgICAgICAoTWF0aC5zaWdudW0ocm90YXRpb25EZWdyZWUpICogcmVsYXRpdmVQdC54KSk7XHJcblxyXG4gICAgdGhpcy5zZXRDZW50ZXIocm90YXRlZFB0LnggKyBvcmlnaW4ueCwgcm90YXRlZFB0LnkgKyBvcmlnaW4ueSk7XHJcblxyXG4gICAgdmFyIG5ld0hlaWdodCA9IHRoaXMuZ2V0V2lkdGgoKTtcclxuICAgIHZhciBuZXdXaWR0aCA9IHRoaXMuZ2V0SGVpZ2h0KCk7XHJcbiAgICB0aGlzLnNldFdpZHRoKG5ld1dpZHRoKTtcclxuICAgIHRoaXMuc2V0SGVpZ2h0KG5ld0hlaWdodCk7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgaXMgdXNlZCBmb3IgcG9ydCBub2RlcyBvbmx5XHJcbiAqL1xyXG5TYmduUEROb2RlLnByb3RvdHlwZS5jYWxjQXZlcmFnZVBvaW50ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGF2ZXJhZ2VQbnQgPSBuZXcgUG9pbnREKDAuMCwgMC4wKTtcclxuICAgIFxyXG4gICAgdmFyIG51bU9mRWRnZXMgPSB0aGlzLmdldEVkZ2VzKCkubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaW5kZXg9MDsgaW5kZXg8bnVtT2ZFZGdlczsgaW5kZXgrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMuZ2V0RWRnZXMoKVtpbmRleF07XHJcbiAgICAgICAgXHJcbiAgICAgICAgaWYgKGVkZ2UudHlwZS5sb2NhbGVDb21wYXJlKFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFKSA9PT0gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuICAgICAgICBcclxuICAgICAgICBhdmVyYWdlUG50LnggKz0gZWRnZS5nZXRPdGhlckVuZCh0aGlzKS5nZXRDZW50ZXJYKCk7XHJcbiAgICAgICAgYXZlcmFnZVBudC55ICs9IGVkZ2UuZ2V0T3RoZXJFbmQodGhpcykuZ2V0Q2VudGVyWSgpO1xyXG4gICAgfVxyXG5cclxuICAgIGF2ZXJhZ2VQbnQueCAvPSAodGhpcy5nZXRFZGdlcygpLmxlbmd0aCAtIDEpO1xyXG4gICAgYXZlcmFnZVBudC55IC89ICh0aGlzLmdldEVkZ2VzKCkubGVuZ3RoIC0gMSk7XHJcblxyXG4gICAgcmV0dXJuIGF2ZXJhZ2VQbnQ7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCByZXR1cm5zIHRoZSBuZWlnaGJvcnMgb2YgYSBnaXZlbiBub2RlLiBOb3RpY2UgdGhhdCB0aGUgZ3JhcGhcclxuKiBpcyBkaXJlY3RlZC4gVGhlcmVmb3JlIGVkZ2VzIHNob3VsZCBoYXZlIHRoZSBnaXZlbiBub2RlIGFzIHRoZSBzb3VyY2VcclxuKiBub2RlLlxyXG4qL1xyXG5TYmduUEROb2RlLnByb3RvdHlwZS5nZXRDaGlsZHJlbk5laWdoYm9ycyA9IGZ1bmN0aW9uICgvKlN0cmluZyovIGVkZ2VUeXBlKVxyXG57XHJcbiAgICB2YXIgbmVpZ2hib3JzID0gW107XHJcblxyXG4gICAgZm9yICh2YXIgaW5kZXggPSAwOyBpbmRleDx0aGlzLmdldEVkZ2VzKCkubGVuZ3RoOyBpbmRleCsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBlZGdlID0gdGhpcy5nZXRFZGdlcygpW2luZGV4XTtcclxuXHJcbiAgICAgICAgaWYgKChlZGdlLmdldFNvdXJjZSgpPT0gdGhpcykgJiYgKGVkZ2UuZ2V0VGFyZ2V0KCkgIT0gdGhpcykpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgbm9kZSA9IGVkZ2UuZ2V0VGFyZ2V0KCk7XHJcblxyXG4gICAgICAgICAgICBpZiAoZWRnZVR5cGUgIT0gbnVsbCAmJiBlZGdlLnR5cGUubG9jYWxlQ29tcGFyZShlZGdlVHlwZSkgPT09IDApXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIG5laWdoYm9ycy5wdXNoKG5vZGUpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGlmIChlZGdlVHlwZSA9PSBudWxsKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBuZWlnaGJvcnMucHVzaChub2RlKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIHJldHVybiBuZWlnaGJvcnM7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgdXBkYXRlcyB0aGUgYm91bmRzIG9mIHRoaXMgY29tcG91bmQgbm9kZS4gSWYgdGhlIG5vZGUgaXMgYVxyXG4gKiBkdW1teSBjb21wb3VuZCwgZG8gbm90IGluY2x1ZGUgbGFiZWwgYW5kIGV4dHJhIG1hcmdpbnMuXHJcbiAqL1xyXG5TYmduUEROb2RlLnByb3RvdHlwZS51cGRhdGVCb3VuZHMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICAvLyBUT0RPOiBEbyB3ZSBuZWVkIGhhbmRsZSBhc3NlcnRpb25zP1xyXG4gICAgLyphc3NlcnQgdGhpcy5nZXRDaGlsZCgpICE9IG51bGw7Ki9cclxuXHJcbiAgICBpZiAodGhpcy5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoICE9PSAwKVxyXG4gICAge1xyXG4gICAgICAgIC8vIHdyYXAgdGhlIGNoaWxkcmVuIG5vZGVzIGJ5IHJlLWFycmFuZ2luZyB0aGUgYm91bmRhcmllc1xyXG4gICAgICAgIHZhciBjaGlsZEdyYXBoID0gdGhpcy5nZXRDaGlsZCgpO1xyXG4gICAgICAgIGNoaWxkR3JhcGgudXBkYXRlQm91bmRzKHRydWUpO1xyXG5cclxuICAgICAgICB0aGlzLnJlY3QueCA9IGNoaWxkR3JhcGguZ2V0TGVmdCgpO1xyXG4gICAgICAgIHRoaXMucmVjdC55ID0gY2hpbGRHcmFwaC5nZXRUb3AoKTtcclxuXHJcbiAgICAgICAgaWYgKCh0aGlzLnR5cGUgIT0gbnVsbCkgJiYgXHJcbiAgICAgICAgICAgICh0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuRFVNTVlfQ09NUE9VTkQpID09PSAwKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuc2V0V2lkdGgoY2hpbGRHcmFwaC5nZXRSaWdodCgpIC0gY2hpbGRHcmFwaC5nZXRMZWZ0KCkpO1xyXG4gICAgICAgICAgICB0aGlzLnNldEhlaWdodChjaGlsZEdyYXBoLmdldEJvdHRvbSgpIC0gY2hpbGRHcmFwaC5nZXRUb3AoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuc2V0V2lkdGgoY2hpbGRHcmFwaC5nZXRSaWdodCgpIC0gY2hpbGRHcmFwaC5nZXRMZWZ0KCkgKyAyXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAqIFNiZ25QRENvbnN0YW50cy5DT01QT1VORF9OT0RFX01BUkdJTik7XHJcbiAgICAgICAgICAgIHRoaXMuc2V0SGVpZ2h0KGNoaWxkR3JhcGguZ2V0Qm90dG9tKCkgLSBjaGlsZEdyYXBoLmdldFRvcCgpICsgMlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKiBTYmduUERDb25zdGFudHMuQ09NUE9VTkRfTk9ERV9NQVJHSU5cclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLkxBQkVMX0hFSUdIVCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG4gICAgICAgIFxyXG5tb2R1bGUuZXhwb3J0cyA9IFNiZ25QRE5vZGU7XHJcbiIsInZhciBJR2VvbWV0cnkgPSByZXF1aXJlKCcuL0lHZW9tZXRyeScpO1xyXG52YXIgUG9pbnREID0gcmVxdWlyZSgnLi9Qb2ludEQnKTtcclxuXHJcbnZhciBTYmduUEROb2RlID0gcmVxdWlyZSgnLi9TYmduUEROb2RlJyk7XHJcbnZhciBTYmduUERFZGdlID0gcmVxdWlyZSgnLi9TYmduUERFZGdlJyk7XHJcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5PcmllbnRhdGlvbkVudW0gPSBcclxue1xyXG4gICAgQk9UVE9NX1RPX1RPUCA6IDAsIFxyXG4gICAgVE9QX1RPX0JPVFRPTSA6IDEsXHJcbiAgICBMRUZUX1RPX1JJR0hUIDogMixcclxuICAgIFJJR0hUX1RPX0xFRlQgOiAzXHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLlJvdGF0aW9uUHJpb3JpdHlFbnVtID0gXHJcbntcclxuICAgIE5JTkVUWV9ERUdSRUUgOiAwLCBcclxuICAgIFNXQVAgOiAxLFxyXG4gICAgTk9fUk9UQVRJT04gOiAyXHJcbn07XHJcblxyXG5mdW5jdGlvbiBTYmduUHJvY2Vzc05vZGUoZ20sIGxvYywgc2l6ZSwgdk5vZGUsIHR5cGUpIFxyXG57XHJcbiAgICBTYmduUEROb2RlLmNhbGwodGhpcywgZ20sIGxvYywgc2l6ZSwgdk5vZGUsIHR5cGUpO1xyXG5cclxuICAgIHRoaXMubmV0Um90YXRpb25hbEZvcmNlID0gMDtcclxuICAgIHRoaXMuY29uc3VtcHRpb25FZGdlcyA9IFtdO1xyXG4gICAgdGhpcy5wcm9kdWN0RWRnZXMgPSBbXTtcclxuICAgIHRoaXMuZWZmZWN0b3JFZGdlcyA9IFtdO1xyXG4gICAgXHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kID0gbnVsbDtcclxuICAgIHRoaXMuaW5wdXRQb3J0ID0gbnVsbDtcclxuICAgIHRoaXMub3V0cHV0UG9ydCA9IG51bGw7XHJcbiAgICBcclxuICAgIHRoaXMub3JpZW50YXRpb24gPSBudWxsO1xyXG4gICAgdGhpcy5yb3RhdGlvblByaW9yaXR5ID0gbnVsbDtcclxuICAgIFxyXG4gICAgdGhpcy5pZGVhbEVkZ2VMZW5ndGggPSAwLjA7XHJcbiAgICB0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSA9IDAuMDtcclxuICAgIHRoaXMucHJvcGVyRWRnZUNvdW50ID0gMC4wO1xyXG59XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShTYmduUEROb2RlLnByb3RvdHlwZSk7XHJcbmZvciAodmFyIHByb3AgaW4gU2JnblBETm9kZSkge1xyXG4gIFNiZ25Qcm9jZXNzTm9kZVtwcm9wXSA9IFNiZ25QRE5vZGVbcHJvcF07XHJcbn1cclxuXHJcbi8qKlxyXG4qIENvbm5lY3QgdGhlIHBvcnQgbm9kZSB0byBpdHMgcHJvY2VzcyBub2RlIChwYXJlbnQpIGFuZCBjb25uZWN0IHRoZSBlZGdlc1xyXG4qIG9mIG5laWdoYm9yIG5vZGVzIHRvIHRoZSBwb3J0IG5vZGUgYnkgY29uc2lkZXJpbmcgdGhlaXIgdHlwZXMgKGZvciBib3RoXHJcbiogaW5wdXQgcG9ydCBhbmQgb3V0cHV0IHBvcnQpXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUucmVjb25uZWN0RWRnZXMgPSBmdW5jdGlvbiAoaWRlYWxFZGdlTGVuZ3RoKVxyXG57XHJcbiAgICB0aGlzLmlkZWFsRWRnZUxlbmd0aCA9IGlkZWFsRWRnZUxlbmd0aDtcclxuICAgIFxyXG4gICAgLy8gY2hhbmdlIGNvbm5lY3Rpb25zIGZyb20gcHJvY2VzcyBub2RlJm5laWdoYm9ycyB0byBwb3J0Jm5laWdoYm9ycy5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy5nZXRFZGdlcygpLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzRWRnZSA9IHRoaXMuZ2V0RWRnZXMoKVtpXTtcclxuXHJcbiAgICAgICAgaWYgKHNFZGdlLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5DT05TVU1QVElPTilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuZ2V0RWRnZXMoKS5zcGxpY2UodGhpcy5nZXRFZGdlcygpLmluZGV4T2Yoc0VkZ2UpLCAxKTtcclxuXHJcbiAgICAgICAgICAgIHNFZGdlLnNldFRhcmdldCh0aGlzLmlucHV0UG9ydCk7XHJcbiAgICAgICAgICAgIHRoaXMuaW5wdXRQb3J0LmdldEVkZ2VzKCkucHVzaChzRWRnZSk7XHJcbiAgICAgICAgICAgIGktLTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAoc0VkZ2UudHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLlBST0RVQ1RJT04pXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLmdldEVkZ2VzKCkuc3BsaWNlKHRoaXMuZ2V0RWRnZXMoKS5pbmRleE9mKHNFZGdlKSwgMSk7XHJcblxyXG4gICAgICAgICAgICBzRWRnZS5zZXRTb3VyY2UodGhpcy5vdXRwdXRQb3J0KTtcclxuICAgICAgICAgICAgdGhpcy5vdXRwdXRQb3J0LmdldEVkZ2VzKCkucHVzaChzRWRnZSk7XHJcbiAgICAgICAgICAgIGktLTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAoc0VkZ2UuaXNFZmZlY3RvcigpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGhpcy5nZXRFZGdlcygpLnNwbGljZSh0aGlzLmdldEVkZ2VzKCkuaW5kZXhPZihzRWRnZSksIDEpO1xyXG5cclxuICAgICAgICAgICAgc0VkZ2Uuc2V0VGFyZ2V0KHRoaXMucGFyZW50Q29tcG91bmQpO1xyXG4gICAgICAgICAgICB0aGlzLnBhcmVudENvbXBvdW5kLmdldEVkZ2VzKCkucHVzaChzRWRnZSk7XHJcbiAgICAgICAgICAgIGktLTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNvbm5lY3ROb2RlcyA9IGZ1bmN0aW9uIChwYXJlbnRDb21wb3VuZCwgaW5wdXRQb3J0LCBvdXRwdXRQb3J0KVxyXG57XHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kID0gcGFyZW50Q29tcG91bmQ7XHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kLmlzRHVtbXlDb21wb3VuZCA9IHRydWU7XHJcbiAgICB0aGlzLmlucHV0UG9ydCA9IGlucHV0UG9ydDtcclxuICAgIHRoaXMub3V0cHV0UG9ydCA9IG91dHB1dFBvcnQ7XHJcbiAgICB0aGlzLm9yaWVudGF0aW9uID0gdGhpcy5PcmllbnRhdGlvbkVudW0uTEVGVF9UT19SSUdIVDtcclxuXHJcbiAgICAvLyBpbml0aWFsIHBsYWNlbWVudC4gcGxhY2UgaW5wdXQgdG8gdGhlIGxlZnQgb2YgdGhlIHByb2Nlc3Mgbm9kZSxcclxuICAgIC8vIG91dHB1dCB0byB0aGUgcmlnaHRcclxuICAgIG91dHB1dFBvcnQuc2V0Q2VudGVyKHRoaXMuZ2V0Q2VudGVyWCgpICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RILCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuZ2V0Q2VudGVyWSgpKTtcclxuICAgIGlucHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKCkgLSBTYmduUERDb25zdGFudHMuUklHSURfRURHRV9MRU5HVEgsIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICB0aGlzLmdldENlbnRlclkoKSk7XHJcbn07XHJcblxyXG4vKipcclxuKiBDaGVjayBpZiB0aGUgcHJvY2VzcyBpcyBlbGlnaWJsZSBmb3Igcm90YXRpb24uIEZpcnN0IGNoZWNrIGlmIGFcclxuKiAxODAtZGVncmVlIGlzIHBvc3NpYmxlIChhcyBpdCBpcyBtb3JlIGNyaXRpY2FsKS5cclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5pc1JvdGF0aW9uTmVjZXNzYXJ5ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgLy8gbm9ybWFsaXplIHRoZSBhbW91bnQgKHBlciBpdGVyYXRpb24pXHJcbiAgICB0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSAvPSAoU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OQUxfRk9SQ0VfSVRFUkFUSU9OX0NPVU5UICogXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICh0aGlzLmNvbnN1bXB0aW9uRWRnZXMubGVuZ3RoICsgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLnByb2R1Y3RFZGdlcy5sZW5ndGggKyBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuZWZmZWN0b3JFZGdlcy5sZW5ndGgpKTtcclxuXHJcbiAgICBpZiAodGhpcy5pc1N3YXBBdmFpbGFibGUoKSlcclxuICAgIHtcclxuICAgICAgICAgdGhpcy5yb3RhdGlvblByaW9yaXR5ID0gdGhpcy5Sb3RhdGlvblByaW9yaXR5RW51bS5TV0FQO1xyXG4gICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2UgaWYgKE1hdGguYWJzKHRoaXMubmV0Um90YXRpb25hbEZvcmNlKSA+IFNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICB7XHJcbiAgICAgICAgIHRoaXMucm90YXRpb25Qcmlvcml0eSA9IHRoaXMuUm90YXRpb25Qcmlvcml0eUVudW0uTklORVRZX0RFR1JFRTtcclxuICAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgIHRoaXMucm90YXRpb25Qcmlvcml0eSA9IHRoaXMuUm90YXRpb25Qcmlvcml0eUVudW0uTk9fUk9UQVRJT047XHJcbiAgICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIElmIHRoZSBwZXJjZW50YWdlIG9mIG9idHVzZSBhbmdsZXMgZXhjZWVkcyB0aGUgdGhyZXNob2xkLCBzd2FwIGlzXHJcbiogcmVxdWlyZWQuXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuaXNTd2FwQXZhaWxhYmxlID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIG9idHVzZUFuZ2xlQ250ID0gMC4wO1xyXG4gICAgdmFyIGFjdXRlQW5nbGVDbnQgPSAwLjA7XHJcblxyXG4gICAgdmFyIG51bU9mQ29uc3VtcHRpb25FZGdlcyA9IHRoaXMuY29uc3VtcHRpb25FZGdlcy5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZDb25zdW1wdGlvbkVkZ2VzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSB0aGlzLmNvbnN1bXB0aW9uRWRnZXNbaV07XHJcbiAgICAgICAgXHJcbiAgICAgICAgaWYgKE1hdGguYWJzKGVkZ2UuY29ycmVzcG9uZGluZ0FuZ2xlKSA+IDkwKVxyXG4gICAgICAgICAgICBvYnR1c2VBbmdsZUNudCsrO1xyXG4gICAgICAgIGVsc2VcclxuICAgICAgICAgICAgYWN1dGVBbmdsZUNudCsrO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZQcm9kdWN0RWRnZXMgPSB0aGlzLnByb2R1Y3RFZGdlcy5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZQcm9kdWN0RWRnZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMucHJvZHVjdEVkZ2VzW2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIGlmIChNYXRoLmFicyhlZGdlLmNvcnJlc3BvbmRpbmdBbmdsZSkgPiA5MClcclxuICAgICAgICAgICAgICAgIG9idHVzZUFuZ2xlQ250Kys7XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICAgICAgYWN1dGVBbmdsZUNudCsrO1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChvYnR1c2VBbmdsZUNudCAvIChvYnR1c2VBbmdsZUNudCArIGFjdXRlQW5nbGVDbnQpID4gU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzE4MF9ERUdSRUUpXHJcbiAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICBlbHNlXHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2Qgcm90YXRlcyB0aGUgYXNzb2NpYXRlZCBjb21wb3VuZCAoYW5kIGl0cyBjaGlsZHJlbjogcHJvY2Vzc1xyXG4qIGFuZCBwb3J0cykuXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuYXBwbHlSb3RhdGlvbiA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIGlmICh0aGlzLnJvdGF0aW9uUHJpb3JpdHkgPT09IHRoaXMuUm90YXRpb25Qcmlvcml0eUVudW0uTklORVRZX0RFR1JFRSlcclxuICAgIHtcclxuICAgICAgICBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSA+IFNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoOTApO1xyXG4gICAgICAgICAgICAgICAgdGhpcy5vcmllbnRhdGlvbiA9IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQ7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZWxzZSBpZiAobmV0Um90YXRpb25hbEZvcmNlIDwgLVNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkJPVFRPTV9UT19UT1ApXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodGhpcy5uZXRSb3RhdGlvbmFsRm9yY2UgPCAtU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzkwX0RFR1JFRSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5yb3RhdGVDb21wb3VuZCg5MCk7XHJcbiAgICAgICAgICAgICAgICB0aGlzLm9yaWVudGF0aW9uID0gdGhpcy5PcmllbnRhdGlvbkVudW0uTEVGVF9UT19SSUdIVDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlIGlmICh0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSA+IFNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodGhpcy5uZXRSb3RhdGlvbmFsRm9yY2UgPiBTYmduUERDb25zdGFudHMuUk9UQVRJT05fOTBfREVHUkVFKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB0aGlzLnJvdGF0ZUNvbXBvdW5kKDkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHRoaXMubmV0Um90YXRpb25hbEZvcmNlIDwgLVNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSAgdGhpcy5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHRoaXMubmV0Um90YXRpb25hbEZvcmNlIDwgLVNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHRoaXMubmV0Um90YXRpb25hbEZvcmNlID4gU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzkwX0RFR1JFRSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5yb3RhdGVDb21wb3VuZCg5MCk7XHJcbiAgICAgICAgICAgICAgICB0aGlzLm9yaWVudGF0aW9uID0gIHRoaXMuT3JpZW50YXRpb25FbnVtLlRPUF9UT19CT1RUT007XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgZWxzZSBpZiAodGhpcy5yb3RhdGlvblByaW9yaXR5ID09PSB0aGlzLlJvdGF0aW9uUHJpb3JpdHlFbnVtLlNXQVApXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHRlbXBDZW50ZXIgPSB0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXIoKTtcclxuICAgICAgICBcclxuICAgICAgICB0aGlzLmlucHV0UG9ydC5zZXRDZW50ZXIodGhpcy5vdXRwdXRQb3J0LmdldENlbnRlclgoKSwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWSgpKTtcclxuICAgICAgICB0aGlzLm91dHB1dFBvcnQuc2V0Q2VudGVyKHRlbXBDZW50ZXIueCwgdGVtcENlbnRlci55KTtcclxuXHJcbiAgICAgICAgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlRPUF9UT19CT1RUT00pXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QO1xyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkJPVFRPTV9UT19UT1ApXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NO1xyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQpXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUO1xyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubmV0Um90YXRpb25hbEZvcmNlID0gMDtcclxufTtcclxuXHJcblxyXG4vKipcclxuKiBHaXZlbiBhIGNvbXBvdW5kIG5vZGUsIHRoaXMgbWV0aG9kIHJlY3Vyc2l2ZWx5IHJvdGF0ZXMgdGhlIGNvbXBvdW5kIG5vZGVcclxuKiBhbmQgaXRzIG1lbWJlcnMuXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUucm90YXRlQ29tcG91bmQgPSBmdW5jdGlvbiAocm90YXRpb25EZWdyZWUpXHJcbntcclxuICAgIHRoaXMucm90YXRlTm9kZSh0aGlzLmdldENlbnRlcigpLCByb3RhdGlvbkRlZ3JlZSk7XHJcbiAgICB0aGlzLmlucHV0UG9ydC5yb3RhdGVOb2RlKHRoaXMuZ2V0Q2VudGVyKCksIHJvdGF0aW9uRGVncmVlKTtcclxuICAgIHRoaXMub3V0cHV0UG9ydC5yb3RhdGVOb2RlKHRoaXMuZ2V0Q2VudGVyKCksIHJvdGF0aW9uRGVncmVlKTtcclxuICAgIHRoaXMucGFyZW50Q29tcG91bmQudXBkYXRlQm91bmRzKCk7XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNhbGNSb3RhdGlvbmFsRm9yY2VzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdGhpcy5uZXRSb3RhdGlvbmFsRm9yY2UgKz0gdGhpcy5jYWxjUHJvcGVybHlPcmllbnRlZEVkZ2VzKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBjYWxjdWxhdGVzIGFsbCBhbmdsZXMgYmV0d2VlbiBwcm9jZXNzIGFuZCBpdHMgZWRnZXMgKHByb2QsXHJcbiogY29ucywgZWZmKSBhbmQgbWFya3MgdGhlbSBhcyBwcm9wZXJseSBvcmllbnRlZCBvciBub3QuIFJldHVybmVkIHZhbHVlIGlzXHJcbiogdGhlIGFtb3VudCBvZiBkZXNpcmUgdG8gcm90YXRlIGF0IHRoaXMgc3RlcC4gVGhlIHJldHVybmVkIHZhbHVlIHNob3VsZCBiZVxyXG4qIHRoZW4gbWFudWFsbHkgYWRkZWQgdG8gbmV0Um90YXRpb25hbEZvcmNlIChpZiBhaW0gaXMgdG8gY2FsY3VsYXRlXHJcbiogbmV0cm90YXRpb25hbGZvcmNlKVxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNhbGNQcm9wZXJseU9yaWVudGVkRWRnZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgaW5wdXRSb3RTdW0gPSAwO1xyXG4gICAgdmFyIG91dHB1dFJvdFN1bSA9IDA7XHJcbiAgICB2YXIgZWZmZWN0b3JSb3RTdW0gPSAwO1xyXG4gICAgdmFyIHN0ZXBTdW0gPSAwO1xyXG4gICAgdmFyIHJlc3VsdDtcclxuICAgIHRoaXMucHJvcGVyRWRnZUNvdW50ID0gMDtcclxuXHJcbiAgICAvLyBpZiB0aGUgbmVpZ2hib3JzIG9mIHBvcnQgbm9kZXMgaGF2ZSBub3QgYmVlbiBkZXRlY3RlZCB5ZXQsIGZpbmQgdGhlbS5cclxuICAgIGlmICh0aGlzLmNvbnN1bXB0aW9uRWRnZXMubGVuZ3RoID09PSAwICYmIHRoaXMucHJvZHVjdEVkZ2VzLmxlbmd0aCA9PT0gMClcclxuICAgIHtcclxuICAgICAgICB0aGlzLmluaXRMaXN0cygpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGZpbmQgaWRlYWwgcG9zaXRpb25zXHJcbiAgICB2YXIgaW5wdXRQb3J0VGFyZ2V0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KHRydWUsIHRoaXMub3JpZW50YXRpb24pO1xyXG4gICAgdmFyIG91dHB1dFBvcnRUYXJnZXQgPSB0aGlzLmZpbmRQb3J0VGFyZ2V0UG9pbnQoZmFsc2UsIHRoaXMub3JpZW50YXRpb24pO1xyXG5cclxuICAgIGZvciAodmFyIG5vZGVJbmRleCA9IDA7IG5vZGVJbmRleCA8IHRoaXMuY29uc3VtcHRpb25FZGdlcy5sZW5ndGg7IG5vZGVJbmRleCsrKVxyXG4gICAge1xyXG4gICAgICAgIHJlc3VsdCA9IHRoaXMuY2FsY1JvdGF0aW9uYWxGb3JjZSh0cnVlLCBub2RlSW5kZXgsIGlucHV0UG9ydFRhcmdldCk7XHJcbiAgICAgICAgaWYgKE1hdGguYWJzKHJlc3VsdCkgPD0gU2JnblBEQ29uc3RhbnRzLkFOR0xFX1RPTEVSQU5DRSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvcGVyRWRnZUNvdW50Kys7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGlucHV0Um90U3VtICs9IHJlc3VsdDtcclxuICAgIH1cclxuICAgIGZvciAodmFyIG5vZGVJbmRleCA9IDA7IG5vZGVJbmRleCA8IHRoaXMucHJvZHVjdEVkZ2VzLmxlbmd0aDsgbm9kZUluZGV4KyspXHJcbiAgICB7XHJcbiAgICAgICAgcmVzdWx0ID0gdGhpcy5jYWxjUm90YXRpb25hbEZvcmNlKGZhbHNlLCBub2RlSW5kZXgsIG91dHB1dFBvcnRUYXJnZXQpO1xyXG4gICAgICAgIGlmIChNYXRoLmFicyhyZXN1bHQpIDw9IFNiZ25QRENvbnN0YW50cy5BTkdMRV9UT0xFUkFOQ0UpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLnByb3BlckVkZ2VDb3VudCsrO1xyXG4gICAgICAgIH1cclxuICAgICAgICBvdXRwdXRSb3RTdW0gKz0gcmVzdWx0O1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIG5vZGVJbmRleCA9IDA7IG5vZGVJbmRleCA8IHRoaXMuZWZmZWN0b3JFZGdlcy5sZW5ndGg7IG5vZGVJbmRleCsrKVxyXG4gICAge1xyXG4gICAgICAgIHJlc3VsdCA9IHRoaXMuY2FsY0VmZmVjdG9yQW5nbGUobm9kZUluZGV4KTtcclxuICAgICAgICBpZiAoTWF0aC5hYnMocmVzdWx0KSA8PSBTYmduUERDb25zdGFudHMuRUZGRUNUT1JfQU5HTEVfVE9MRVJBTkNFKVxyXG4gICAgICAgICAgICAgICAgdGhpcy5wcm9wZXJFZGdlQ291bnQrKztcclxuXHJcbiAgICAgICAgZWZmZWN0b3JSb3RTdW0gKz0gTWF0aC5hYnMocmVzdWx0KTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBhZGQgdG90YWwgZWZmZWN0b3Igcm90YXRpb25hbCBmb3JjZSB3aXRoIHRoZSBzYW1lIHNpZ24gb2ZcclxuICAgIC8vIHN0ZXAgc3VtIGJlY2F1c2UgaXQgZG9lcyBub3QgbWF0dGVyIGZvciBhbiBlZmZlY3RvciBub2RlXHJcbiAgICAvLyBlaXRoZXIgcm90YXRlIHRvIGxlZnQgb3IgcmlnaHQuIHRoZXJlZm9yZSBzdXBwb3J0IHRoZSByb3RhdGlvblxyXG4gICAgLy8gZGlyZWN0aW9uIG9mIGVhY2ggaXRlcmF0aW9uLlxyXG4gICAgc3RlcFN1bSA9IGlucHV0Um90U3VtIC0gb3V0cHV0Um90U3VtO1xyXG4gICAgc3RlcFN1bSA9IHN0ZXBTdW0gKyAoTWF0aC5zaWduKHN0ZXBTdW0pICogTWF0aC5hYnMoZWZmZWN0b3JSb3RTdW0pKTtcclxuXHJcbiAgICByZXR1cm4gc3RlcFN1bTtcclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIHJldHVybnMgdGhlIHNpZ25lZCBhbmdsZSBiZXR3ZWVuIGEgbm9kZSBhbmQgaXRzIGNvcnJlc3BvbmRpbmdcclxuKiBwb3J0IGFuZCB0aGUgdGFyZ2V0IHBvaW50LlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNhbGNSb3RhdGlvbmFsRm9yY2UgPSBmdW5jdGlvbiAoaXNJbnB1dFBvcnQsIG5vZGVJbmRleCwgdGFyZ2V0UG9pbnQpXHJcbntcclxuICAgIHZhciBub2RlO1xyXG4gICAgdmFyIGNlbnRlclBvaW50O1xyXG5cclxuICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgIHtcclxuICAgICAgICBub2RlID0gdGhpcy5jb25zdW1wdGlvbkVkZ2VzW25vZGVJbmRleF0uZ2V0U291cmNlKCk7XHJcbiAgICAgICAgY2VudGVyUG9pbnQgPSB0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXIoKTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICBub2RlID0gdGhpcy5wcm9kdWN0RWRnZXNbbm9kZUluZGV4XS5nZXRUYXJnZXQoKTtcclxuICAgICAgICBjZW50ZXJQb2ludCA9IHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXIoKTtcclxuICAgIH1cclxuXHJcbiAgICB2YXIgYW5nbGUgPSBJR2VvbWV0cnkuY2FsY3VsYXRlQW5nbGUodGFyZ2V0UG9pbnQsIGNlbnRlclBvaW50LCBub2RlLmdldENlbnRlcigpKTtcclxuXHJcbiAgICBpZiAoaXNJbnB1dFBvcnQpXHJcbiAgICAgICAgYW5nbGUgKj0gdGhpcy5pc0xlZnQodGFyZ2V0UG9pbnQsIGNlbnRlclBvaW50LCBub2RlLmdldENlbnRlcigpLCBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVCk7XHJcbiAgICBlbHNlXHJcbiAgICAgICAgYW5nbGUgKj0gdGhpcy5pc0xlZnQodGFyZ2V0UG9pbnQsIGNlbnRlclBvaW50LCBub2RlLmdldENlbnRlcigpLCBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpO1xyXG5cclxuICAgIHRoaXMuc2F2ZUluZm9ybWF0aW9uKGlzSW5wdXRQb3J0LCBub2RlSW5kZXgsIGFuZ2xlKTtcclxuXHJcbiAgICByZXR1cm4gYW5nbGU7XHJcbn07XHJcblxyXG4vKipcclxuKiBDYWxjdWxhdGVzIHRoZSBhbmdsZSBiZXR3ZWVuIGFuIGVmZmVjdG9yIGVkZ2UgYW5kIGl0cyBwcm9jZXNzIG5vZGUuIEFuXHJcbiogZWZmZWN0b3IgZWRnZSBoYXMgcHJvY2VzcyAoaW4gdGhpcyBjYXNlIHRoZSBkdW1teSBjb21wb3VuZCkgYXMgaXRzIHRhcmdldFxyXG4qIG5vZGUgYW5kIHRoZSBlZmZlY3RvciBpdHNlbGYgYXMgdGhlIHNvdXJjZS5cclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5jYWxjRWZmZWN0b3JBbmdsZSA9IGZ1bmN0aW9uIChub2RlSW5kZXgpXHJcbntcclxuICAgIHZhciBlZmYgPSB0aGlzLmVmZmVjdG9yRWRnZXNbbm9kZUluZGV4XS5nZXRTb3VyY2UoKTtcclxuICAgIHZhciB0YXJnZXRQbnQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICB2YXIgY2VudGVyUG50ID0gdGhpcy5nZXRDZW50ZXIoKTtcclxuXHJcbiAgICAvLyBmaW5kIHRhcmdldCBwb2ludFxyXG4gICAgaWYgKHRoaXMuaXNIb3Jpem9udGFsKCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UG50LnggPSB0aGlzLmdldENlbnRlclgoKTtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJZKCkgPiB0aGlzLmdldENlbnRlclkoKSlcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnkgPSB0aGlzLmdldENlbnRlclkoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgICAgIGVsc2VcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnkgPSB0aGlzLmdldENlbnRlclkoKSAtIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAodGhpcy5pc1ZlcnRpY2FsKCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UG50LnkgPSB0aGlzLmdldENlbnRlclkoKTtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJYKCkgPiB0aGlzLmdldENlbnRlclgoKSlcclxuICAgICAgICAgICAgICAgIHRhcmdldFBudC54ID0gdGhpcy5nZXRDZW50ZXJYKCkgKyB0aGlzLmlkZWFsRWRnZUxlbmd0aDtcclxuICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgICAgICB0YXJnZXRQbnQueCA9IHRoaXMuZ2V0Q2VudGVyWCgpIC0gdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICB9XHJcblxyXG4gICAgdmFyIGFuZ2xlID0gSUdlb21ldHJ5LmNhbGN1bGF0ZUFuZ2xlKHRhcmdldFBudCwgY2VudGVyUG50LCBlZmYuZ2V0Q2VudGVyKCkpO1xyXG5cclxuICAgIHRoaXMuZWZmZWN0b3JFZGdlc1tub2RlSW5kZXhdLmNvcnJlc3BvbmRpbmdBbmdsZSA9IGFuZ2xlO1xyXG5cclxuICAgIGlmIChNYXRoLmFicyhhbmdsZSkgPD0gU2JnblBEQ29uc3RhbnRzLkVGRkVDVE9SX0FOR0xFX1RPTEVSQU5DRSlcclxuICAgICAgICB0aGlzLmVmZmVjdG9yRWRnZXNbbm9kZUluZGV4XS5pc1Byb3Blcmx5T3JpZW50ZWQgPSB0cnVlO1xyXG4gICAgZWxzZVxyXG4gICAgICAgIHRoaXMuZWZmZWN0b3JFZGdlc1tub2RlSW5kZXhdLmlzUHJvcGVybHlPcmllbnRlZCA9IGZhbHNlO1xyXG5cclxuICAgIHJldHVybiBhbmdsZTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuYXBwbHlBcHByb3hpbWF0aW9ucyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIC8vIGlmIHRoZXJlIGlzIG9ubHkgb25lIHNpbmdsZS1lZGdlIGNvbnN1bXB0aW9uLCBtb3ZlIGl0IHRvIGlkZWFsXHJcbiAgICAvLyBvdGhlcndpc2UgbW92ZSB0b3dhcmRzIG11bHRpZWRnZSBub2RlXHJcblxyXG4gICAgaWYgKHRoaXMuY29uc3VtcHRpb25FZGdlcy5sZW5ndGggPT09IDEgJiYgXHJcbiAgICAgICAgdGhpcy5jb25zdW1wdGlvbkVkZ2VzWzBdLmdldFNvdXJjZSgpLmdldEVkZ2VzKCkubGVuZ3RoID09PSAxKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuYXBwcm94aW1hdGVGb3JTaW5nbGVOb2Rlcyh0aGlzLmlucHV0UG9ydCwgdGhpcy5jb25zdW1wdGlvbkVkZ2VzWzBdLmdldFNvdXJjZSgpKTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICB0aGlzLmFwcHJveGltYXRlRm9yTXVsdGlwbGVOb2Rlcyh0aGlzLmlucHV0UG9ydCk7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHRoaXMucHJvZHVjdEVkZ2VzLmxlbmd0aCA9PT0gMSAmJiBcclxuICAgICAgICB0aGlzLnByb2R1Y3RFZGdlc1swXS5nZXRUYXJnZXQoKS5nZXRFZGdlcygpLmxlbmd0aCA9PT0gMSlcclxuICAgIHsgICAgXHJcbiAgICAgICAgdGhpcy5hcHByb3hpbWF0ZUZvclNpbmdsZU5vZGVzKHRoaXMub3V0cHV0UG9ydCwgdGhpcy5wcm9kdWN0RWRnZXNbMF0uZ2V0VGFyZ2V0KCkpO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIGFwcHJveGltYXRlRm9yTXVsdGlwbGVOb2Rlcyh0aGlzLm91dHB1dFBvcnQpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMuYXBwcm94aW1hdGVFZmZlY3RvcnMoKTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuYXBwcm94aW1hdGVGb3JTaW5nbGVOb2RlcyA9IGZ1bmN0aW9uIChwb3J0LCBub2RlKVxyXG57XHJcbiAgICB2YXIgdGFyZ2V0UHQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICB2YXIgbmV3UG9pbnQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICBcclxuICAgIGlmIChwb3J0LmlzSW5wdXRQb3J0KCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UHQgPSB0aGlzLmZpbmRQb3J0VGFyZ2V0UG9pbnQodHJ1ZSwgdGhpcy5vcmllbnRhdGlvbik7XHJcbiAgICB9ICAgIFxyXG4gICAgXHJcbiAgICBlbHNlIGlmIChwb3J0LmlzT3V0cHV0UG9ydCgpKVxyXG4gICAge1xyXG4gICAgICAgIHRhcmdldFB0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KGZhbHNlLCB0aGlzLm9yaWVudGF0aW9uKTsgICBcclxuICAgIH1cclxuICAgIFxyXG4gICAgbmV3UG9pbnQueCA9IHRhcmdldFB0LnhcclxuICAgICAgICAgICAgICAgICAgICArIChNYXRoLnJhbmRvbSgpICogU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0UgKiAyKVxyXG4gICAgICAgICAgICAgICAgICAgIC0gU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0U7XHJcbiAgICBuZXdQb2ludC55ID0gdGFyZ2V0UHQueVxyXG4gICAgICAgICAgICAgICAgICAgICsgKE1hdGgucmFuZG9tKCkgKiBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRSAqIDIpXHJcbiAgICAgICAgICAgICAgICAgICAgLSBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRTtcclxuXHJcbiAgICBub2RlLnNldENlbnRlcihuZXdQb2ludC54LCBuZXdQb2ludC55KTtcclxufTtcclxuXHJcbi8qKlxyXG4qIEdpdmVuIHRoZSBwb3J0IG5vZGUsIHRoaXMgbWV0aG9kIGZpbmRzIGFsbCBjb25zdW1wdGlvbihvciBwcm9kdWN0aW9uKVxyXG4qIG5vZGVzIG9mIHRoaXMgcG9ydCBub2RlLiBUaGUgaWRlYSBpcyB0byBtb3ZlIGVhY2ggc2luZ2xlLWVkZ2VcclxuKiBjb25zdW1wdGlvbnMocHJvZHVjdHMpIG9mIGEgcHJvY2VzcyBjbG9zZXIgdG8gbmVpZ2hib3IgbXVsdGktZWRnZSBub2Rlc1xyXG4qIHRvIGtlZXAgdGhlbSBjbG9zZSB0byBlYWNoIG90aGVyLlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmFwcHJveGltYXRlRm9yTXVsdGlwbGVOb2RlcyA9IGZ1bmN0aW9uIChwb3J0KVxyXG57XHJcbiAgICB2YXIgb25lRWRnZU5vZGVzID0gW107XHJcbiAgICB2YXIgbXVsdGlFZGdlTm9kZXMgPSBbXTtcclxuICAgIHZhciBub2RlT2ZJbnRlcmVzdCA9IG51bGw7XHJcbiAgICB2YXIgdGFyZ2V0UHQgPSBuZXcgUG9pbnREKCk7XHJcblxyXG4gICAgLy8gZ2V0IGFsbCBub24tcmlnaWQgZWRnZXMgb2YgcG9ydCBub2RlXHJcbiAgICB2YXIgbnVtT2ZFZGdlcyA9IHBvcnQuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZFZGdlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBlZGdlID0gcG9ydC5nZXRFZGdlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAoZWRnZS5pc1JpZ2lkRWRnZSgpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBub2RlIG9mIGludGVyZXN0IGRlcGVuZHMgb24gdGhlIGRpcmVjdGlvbiBvZiB0aGUgZWRnZVxyXG4gICAgICAgIGlmIChwb3J0LmlzSW5wdXRQb3J0KCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBub2RlT2ZJbnRlcmVzdCA9IGVkZ2UuZ2V0U291cmNlKCk7ICAgXHJcbiAgICAgICAgfSAgICAgICAgXHJcbiAgICAgICAgZWxzZSBpZiAocG9ydC5pc091dHB1dFBvcnQoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG5vZGVPZkludGVyZXN0ID0gZWRnZS5nZXRUYXJnZXQoKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIGlmIChub2RlT2ZJbnRlcmVzdC5nZXRFZGdlcygpLmxlbmd0aCA9PT0gMSkgLy8gc2luZ2xlIG5vZGUgICAgXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBvbmVFZGdlTm9kZXMucHVzaChub2RlT2ZJbnRlcmVzdCk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKG5vZGVPZkludGVyZXN0LmdldEVkZ2VzKCkubGVuZ3RoID4gMSkgLy8gbXVsdGllZGdlIG5vZGVcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG11bHRpRWRnZU5vZGVzLnB1c2gobm9kZU9mSW50ZXJlc3QpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICBpZiAocG9ydC5pc0lucHV0UG9ydCgpKVxyXG4gICAge1xyXG4gICAgICAgIHRhcmdldFB0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KHRydWUsIHRoaXMub3JpZW50YXRpb24pOyAgIFxyXG4gICAgfSAgICAgICAgXHJcbiAgICBlbHNlIGlmIChwb3J0LmlzT3V0cHV0UG9ydCgpKVxyXG4gICAge1xyXG4gICAgICAgIHRhcmdldFB0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KGZhbHNlLCB0aGlzLm9yaWVudGF0aW9uKTtcclxuICAgIH0gICAgICAgIFxyXG4gICAgXHJcbiAgICAvLyBtb3ZlXHJcbiAgICBpZiAob25lRWRnZU5vZGVzLmxlbmd0aCA+IDApXHJcbiAgICB7XHJcbiAgICAgICAgbW92ZU9uZUVkZ2VOb2RlcyhvbmVFZGdlTm9kZXMsIG11bHRpRWRnZU5vZGVzLCB0YXJnZXRQdCk7XHJcbiAgICB9ICAgIFxyXG59O1xyXG5cclxuLyoqXHJcbiogU2luZ2xlLWVkZ2Ugbm9kZXMgYXJlIG1vdmVkIGFyb3VuZCB0aGUgY2VudGVyIHBvaW50IG9mIGEgbXVsdGktZWRnZSBub2RlLlxyXG4qIElmIGFsbCB0aGUgbmVpZ2hib3Igbm9kZXMgb2YgdGhhdCBwb3J0IG5vZGUgYXJlIHNpbmdsZS1lZGdlZCwgdGhlbiBvbmUgb2ZcclxuKiB0aGVtIGlzIGNob3NlbiByYW5kb21seSBhbmQgdGhlIG90aGVycyBhcmUgcGxhY2VkIGFyb3VuZCBpdC5cclxuKiBcclxuKiBAcGFyYW0gdGFyZ2V0UHRcclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5tb3ZlT25lRWRnZU5vZGVzID0gZnVuY3Rpb24gKG9uZUVkZ2VOb2RlcywgbXVsdGlFZGdlTm9kZXMsIHRhcmdldFB0KVxyXG57XHJcbiAgICB2YXIgYXBwcm94aW1hdGlvblBudCA9IG5ldyBQb2ludEQoMCwgMCk7XHJcbiAgICB2YXIgcmFuZG9tSW5kZXggPSAtMTtcclxuICAgIHZhciBhcHByb3hpbWF0aW9uTm9kZSA9IG51bGw7XHJcbiAgICB2YXIgbmV3UG9pbnQgPSBuZXcgUG9pbnREKCk7XHJcblxyXG4gICAgLy8gaWYgdGhlcmUgYXJlIG1vcmUgdGhhbiBvbmUgbXVsdGkgZWRnZSBub2Rlcywgc2VsZWN0IHRoZSBoaWdobHlcclxuICAgIC8vIGNvbm5lY3RlZCBvbmVcclxuICAgIGlmIChtdWx0aUVkZ2VOb2Rlcy5sZW5ndGggPiAwKVxyXG4gICAge1xyXG4gICAgICAgIGFwcHJveGltYXRpb25Ob2RlID0gbXVsdGlFZGdlTm9kZXNbMF07XHJcbiAgICAgICAgXHJcbiAgICAgICAgdmFyIG51bU9mTXVsdGlFZGdlTm9kZXMgPSBtdWx0aUVkZ2VOb2Rlcy5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgaT0wOyBpPG51bU9mTXVsdGlFZGdlTm9kZXM7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBub2RlID0gbXVsdGlFZGdlTm9kZXNbaV07XHJcblxyXG4gICAgICAgICAgICBpZiAobm9kZS5nZXRFZGdlcygpLmxlbmd0aCA+IGFwcHJveGltYXRpb25Ob2RlLmdldEVkZ2VzKCkubGVuZ3RoKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBhcHByb3hpbWF0aW9uTm9kZSA9IG5vZGU7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gaWYgdGhlcmUgYXJlIG5vIG11bHRpIGVkZ2Ugbm9kZXMsIHJhbmRvbWx5IHNlbGVjdCBvbmVcclxuICAgIGVsc2UgaWYgKG11bHRpRWRnZU5vZGVzLmxlbmd0aCA9PT0gMClcclxuICAgIHtcclxuICAgICAgICByYW5kb21JbmRleCA9IChNYXRoLnJhbmRvbSgpICogb25lRWRnZU5vZGVzLmxlbmd0aCk7XHJcbiAgICAgICAgYXBwcm94aW1hdGlvbk5vZGUgPSBvbmVFZGdlTm9kZXNbcmFuZG9tSW5kZXhdO1xyXG4gICAgfVxyXG5cclxuICAgIGFwcHJveGltYXRpb25QbnQgPSBhcHByb3hpbWF0aW9uTm9kZS5nZXRDZW50ZXIoKTtcclxuXHJcbiAgICAvLyBtb3ZlIHNpbmdsZSBub2RlcyBhcm91bmQgdGhlIGFwcHJveGltYXRpb24gcG9pbnRcclxuICAgIHZhciBudW1PZk9uZUVkZ2VOb2RlcyA9IG9uZUVkZ2VOb2Rlcy5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZPbmVFZGdlTm9kZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgcyA9IG9uZUVkZ2VOb2Rlc1tpXTtcclxuICAgICAgICBcclxuICAgICAgICAvLyBpZiB0aGV5IGJlbG9uZyB0byBkaWZmZXJlbnQgZ3JhcGhzLCBkbyBub3QgbW92ZVxyXG4gICAgICAgIC8vIGlmIChhcHByb3hpbWF0aW9uTm9kZS5nZXRPd25lcigpICE9IHMuZ2V0T3duZXIoKSlcclxuICAgICAgICAvLyBjb250aW51ZTtcclxuICAgICAgICBuZXdQb2ludC54ID0gYXBwcm94aW1hdGlvblBudC54XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgKE1hdGgucmFuZG9tKCkgKiBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRSAqIDIpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC0gU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0U7XHJcbiAgICAgICAgbmV3UG9pbnQueSA9IGFwcHJveGltYXRpb25QbnQueVxyXG4gICAgICAgICAgICAgICAgICAgICAgICArIChNYXRoLnJhbmRvbSgpICogU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0UgKiAyKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX0RJU1RBTkNFO1xyXG5cclxuICAgICAgICBzLnNldENlbnRlcihuZXdQb2ludC54LCBuZXdQb2ludC55KTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIElkZW50aWZ5IHNpbmdsZS1lZGdlIGVmZmVjdG9ycy4gTm90ZSB0aGF0IGEgcHJvY2VzcyBtYXkgaGF2ZSBhIG51bWJlciBvZlxyXG4qIGVmZmVjdG9ycy4gRmluZCB0aGUgbG9jYXRpb24gb2YgZWFjaCBlZmZlY3Rvci4gSWYgdGhlIG9yaWVudGF0aW9uIG9mXHJcbiogcHJvY2VzcyBub2RlIGlzIHZlcnRpY2FsLCBpZGVhbCBwb3NpdGlvbiBvZiBlZmZlY3RvcnMgaXMgb24gdGhlXHJcbiogaG9yaXpvbnRhbCBkaXJlY3Rpb25zLiAob3IgdmljZSB2ZXJzYSlcclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5hcHByb3hpbWF0ZUVmZmVjdG9ycyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBuZXdQb2ludCA9IG5ldyBQb2ludEQoKTtcclxuICAgIHZhciBhcHByb3hpbWF0aW9uUG50ID0gbmV3IFBvaW50RCgpO1xyXG5cclxuICAgIC8vIGlkZW50aWZ5IHRoZSBlZmZlY3RvcnNcclxuICAgIHZhciBudW1PZkVmZmVjdG9yRWRnZXMgPSB0aGlzLmVmZmVjdG9yRWRnZXMubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaT0wOyBpPG51bU9mRWZmZWN0b3JFZGdlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBlZGdlID0gdGhpcy5lZmZlY3RvckVkZ2VzW2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIC8vIHNvdXJjZSBub2RlIG9mIGVhY2ggZWZmZWN0b3IgZWRnZSBpcyB0aGUgZWZmZWN0b3IgaXRzZWxmLiBvbmx5XHJcbiAgICAgICAgLy8gbW92ZSBzaW5nbGUgZWZmZWN0b3JzXHJcbiAgICAgICAgaWYgKGVkZ2UuZ2V0U291cmNlKCkuZ2V0RWRnZXMoKS5sZW5ndGggIT09IDEpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIGFwcHJveGltYXRpb25QbnQgPSB0aGlzLmZpbmRFZmZlY3RvclRhcmdldFBvaW50KGVkZ2UuZ2V0U291cmNlKCkpO1xyXG5cclxuICAgICAgICAvLyBwbGFjZSBlZmZlY3RvciBpbiBhIGNpcmN1bGFyIGFyZWEgdXNpbmcgc29tZSByYW5kb21uZXNzXHJcbiAgICAgICAgbmV3UG9pbnQueCA9IGFwcHJveGltYXRpb25QbnQueFxyXG4gICAgICAgICAgICAgICAgICAgICAgICArIChNYXRoLnJhbmRvbSgpICogU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0UgKiAyKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX0RJU1RBTkNFO1xyXG4gICAgICAgIG5ld1BvaW50LnkgPSBhcHByb3hpbWF0aW9uUG50LnlcclxuICAgICAgICAgICAgICAgICAgICAgICAgKyAoTWF0aC5yYW5kb20oKSAqIFNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX0RJU1RBTkNFICogMilcclxuICAgICAgICAgICAgICAgICAgICAgICAgLSBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRTtcclxuXHJcbiAgICAgICAgZWRnZS5nZXRTb3VyY2UoKS5zZXRDZW50ZXIobmV3UG9pbnQueCwgbmV3UG9pbnQueSk7XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBHaXZlbiB0aGUgZWZmZWN0b3IgYW5kIGl0cyBjb3JyZXNwb25kaW5nIHByb2Nlc3MsIHRoZSBtZXRob2QgcmV0dXJucyB0aGVcclxuKiBpZGVhbCBwb3NpdGlvbiBvZiB0aGUgZWZmZWN0b3Igbm9kZSwgd2hpY2ggaGFzIGEgZGlzdGFuY2Ugb2YgaWRlYWwgZWRnZVxyXG4qIGxlbmd0aCBmcm9tIGl0cyBwcm9jZXNzLlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmZpbmRFZmZlY3RvclRhcmdldFBvaW50ID0gZnVuY3Rpb24gKGVmZilcclxue1xyXG4gICAgdmFyIGFwcHJveGltYXRpb25QbnQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICBcclxuICAgIC8vIGZpbmQgdGFyZ2V0IHBvaW50XHJcbiAgICBpZiAodGhpcy5pc0hvcml6b250YWwoKSlcclxuICAgIHtcclxuICAgICAgICBhcHByb3hpbWF0aW9uUG50LnggPSB0aGlzLmdldENlbnRlclgoKTtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJZKCkgPiB0aGlzLmdldENlbnRlclkoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGFwcHJveGltYXRpb25QbnQueSA9IHRoaXMuZ2V0Q2VudGVyWSgpICsgdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGFwcHJveGltYXRpb25QbnQueSA9IHRoaXMuZ2V0Q2VudGVyWSgpIC0gdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAodGhpcy5pc1ZlcnRpY2FsKCkpXHJcbiAgICB7XHJcbiAgICAgICAgYXBwcm94aW1hdGlvblBudC55ID0gdGhpcy5nZXRDZW50ZXJZKCk7XHJcblxyXG4gICAgICAgIGlmIChlZmYuZ2V0Q2VudGVyWCgpID4gdGhpcy5nZXRDZW50ZXJYKCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBhcHByb3hpbWF0aW9uUG50LnggPSB0aGlzLmdldENlbnRlclgoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBhcHByb3hpbWF0aW9uUG50LnggPSB0aGlzLmdldENlbnRlclgoKSAtIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gYXBwcm94aW1hdGlvblBudDtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuZmluZFBvcnRUYXJnZXRQb2ludCA9IGZ1bmN0aW9uIChpc0lucHV0UG9ydCwgb3JpZW50KVxyXG57XHJcbiAgICBpZiAob3JpZW50ID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKCh0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXJYKCkgLSB0aGlzLmlkZWFsRWRnZUxlbmd0aCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclkoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKCh0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWCgpICsgdGhpcy5pZGVhbEVkZ2VMZW5ndGgpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWSgpKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmIChvcmllbnQgPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKGlzSW5wdXRQb3J0KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBQb2ludEQoKHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclgoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy5pbnB1dFBvcnQuZ2V0Q2VudGVyWSgpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBQb2ludEQoKHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXJYKCkgLSB0aGlzLmlkZWFsRWRnZUxlbmd0aCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXJZKCkpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIGVsc2UgaWYgKG9yaWVudCA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTSlcclxuICAgIHtcclxuICAgICAgICBpZiAoaXNJbnB1dFBvcnQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXJYKCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICh0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXJZKCkgLSB0aGlzLmlkZWFsRWRnZUxlbmd0aCkpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWCgpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAodGhpcy5vdXRwdXRQb3J0LmdldENlbnRlclkoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAob3JpZW50ID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclgoKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgKHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclkoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXJYKCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICh0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWSgpIC0gdGhpcy5pZGVhbEVkZ2VMZW5ndGgpKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIG51bGw7XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmluaXRMaXN0cyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBudW1PZkNvbnN1bXB0aW9uRWRnZXMgPSB0aGlzLmlucHV0UG9ydC5nZXRFZGdlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNvbnN1bXB0aW9uRWRnZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMuaW5wdXRQb3J0LmdldEVkZ2VzKClbaV07XHJcbiAgICAgICAgXHJcbiAgICAgICAgaWYgKCFlZGdlLmlzUmlnaWRFZGdlKCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLmNvbnN1bXB0aW9uRWRnZXMuYWRkKGVkZ2UpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICB2YXIgbnVtT2ZQcm9kdWN0RWRnZXMgPSB0aGlzLm91dHB1dFBvcnQuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZQcm9kdWN0RWRnZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMub3V0cHV0UG9ydC5nZXRFZGdlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAoIWVkZ2UuaXNSaWdpZEVkZ2UoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvZHVjdEVkZ2VzLmFkZChlZGdlKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZGV0ZWN0IGFsbCBlZmZlY3RvciBub2RlcyBjb25uZWN0ZWQgdG8gdGhpcyBwcm9jZXNzIG5vZGUgKGlmXHJcbiAgICAvLyB0aGVyZSBhcmUgYW55KVxyXG4gICAgdmFyIG51bU9mRWZmZWN0b3JFZGdlcyA9IHRoaXMucGFyZW50Q29tcG91bmQuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZFZmZlY3RvckVkZ2VzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSB0aGlzLnBhcmVudENvbXBvdW5kLmdldEVkZ2VzKClbaV07XHJcblxyXG4gICAgICAgIGlmIChlZGdlLmlzRWZmZWN0b3IoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuZWZmZWN0b3JFZGdlcy5hZGQoZWRnZSk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5zYXZlSW5mb3JtYXRpb24gPSBmdW5jdGlvbiAoaXNJbnB1dFBvcnQsIG5vZGVJbmRleCwgYW5nbGUpXHJcbntcclxuXHJcbiAgICAvLyByZW1lbWJlciBhbmdsZXMgZXNwZWNpYWxseSBmb3IgZGVidWcgcHVycG9zZXNcclxuICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgIHtcclxuICAgICAgICB0aGlzLmNvbnN1bXB0aW9uRWRnZXNbbm9kZUluZGV4XS5jb3JyZXNwb25kaW5nQW5nbGUgPSBhbmdsZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICB0aGlzLnByb2R1Y3RFZGdlc1tub2RlSW5kZXhdLmNvcnJlc3BvbmRpbmdBbmdsZSA9IGFuZ2xlO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIG5vdGUgaWYgdGhlIGVkZ2VzIGFyZSBwcm9wZXJseSBvcmllbnRlZFxyXG4gICAgaWYgKE1hdGguYWJzKGFuZ2xlKSA8PSBTYmduUERDb25zdGFudHMuQU5HTEVfVE9MRVJBTkNFKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuY29uc3VtcHRpb25FZGdlc1tub2RlSW5kZXhdLmlzUHJvcGVybHlPcmllbnRlZCA9IHRydWU7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvZHVjdEVkZ2VzW25vZGVJbmRleF0uaXNQcm9wZXJseU9yaWVudGVkID0gdHJ1ZTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKGlzSW5wdXRQb3J0KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGhpcy5jb25zdW1wdGlvbkVkZ2VzW25vZGVJbmRleF0uaXNQcm9wZXJseU9yaWVudGVkID0gZmFsc2U7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvZHVjdEVkZ2VzW25vZGVJbmRleF0uaXNQcm9wZXJseU9yaWVudGVkID0gZmFsc2U7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5pc0xlZnQgPSBmdW5jdGlvbiAoYSwgYiwgYywgdHlwZSlcclxue1xyXG4gICAgaWYgKCgoYi54IC0gYS54KSAqIChjLnkgLSBhLnkpIC0gKGIueSAtIGEueSkgKiAoYy54IC0gYS54KSkgPiAwKVxyXG4gICAge1xyXG4gICAgICAgIC8vIGxlZnQgdHVyblxyXG4gICAgICAgIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIC0xO1xyXG4gICAgICAgICAgICBlbHNlIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gMTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAxO1xyXG4gICAgICAgICAgICBlbHNlIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gLTE7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLklOUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gMTtcclxuICAgICAgICAgICAgZWxzZSBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLk9VVFBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIC0xO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIC0xO1xyXG4gICAgICAgICAgICBlbHNlIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gMTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgLy8gcmlnaHQgdHVyblxyXG4gICAgICAgIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDE7XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAtMTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAtMTtcclxuICAgICAgICAgICAgZWxzZSBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLk9VVFBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDE7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLklOUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gLTE7XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAxO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDE7XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAtMTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICByZXR1cm4gMDtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuY29weU5vZGUgPSBmdW5jdGlvbiAocywgZ3JhcGhNYW5hZ2VyKSAvLyBUT0RPOiBncmFwaE1hbmFnZXIgaXMgbm90IHVzZWQuXHJcbntcclxuICAgIHRoaXMudHlwZSA9IHMudHlwZTtcclxuICAgIHRoaXMubGFiZWwgPSBzLmxhYmVsO1xyXG4gICAgdGhpcy5wYXJlbnRDb21wb3VuZCA9IHMucGFyZW50Q29tcG91bmQ7XHJcbiAgICB0aGlzLmlucHV0UG9ydCA9IHMuaW5wdXRQb3J0O1xyXG4gICAgdGhpcy5vdXRwdXRQb3J0ID0gcy5vdXRwdXRQb3J0O1xyXG4gICAgdGhpcy5zZXRDZW50ZXIocy5nZXRDZW50ZXJYKCksIHMuZ2V0Q2VudGVyWSgpKTtcclxuICAgIHRoaXMuc2V0Q2hpbGQocy5nZXRDaGlsZCgpKTtcclxuICAgIHRoaXMuc2V0SGVpZ2h0KHMuZ2V0SGVpZ2h0KCkpO1xyXG4gICAgdGhpcy5zZXRMb2NhdGlvbihzLmdldExvY2F0aW9uKCkueCwgcy5nZXRMb2NhdGlvbigpLnkpO1xyXG4gICAgdGhpcy5zZXROZXh0KHMuZ2V0TmV4dCgpKTtcclxuICAgIHRoaXMuc2V0T3duZXIocy5nZXRPd25lcigpKTtcclxuICAgIHRoaXMuc2V0UHJlZDEocy5nZXRQcmVkMSgpKTtcclxuICAgIHRoaXMuc2V0UHJlZDIocy5nZXRQcmVkMigpKTtcclxuICAgIHRoaXMuc2V0V2lkdGgocy5nZXRXaWR0aCgpKTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuY29weUZyb21TQkdOUEROb2RlID0gZnVuY3Rpb24gKHMsIGdyYXBoTWFuYWdlcilcclxue1xyXG4gICAgdGhpcy50eXBlID0gcy50eXBlO1xyXG4gICAgdGhpcy5sYWJlbCA9IHMubGFiZWw7XHJcbiAgICB0aGlzLnNldENlbnRlcihzLmdldENlbnRlclgoKSwgcy5nZXRDZW50ZXJZKCkpO1xyXG4gICAgdGhpcy5zZXRDaGlsZChzLmdldENoaWxkKCkpO1xyXG4gICAgdGhpcy5zZXRIZWlnaHQocy5nZXRIZWlnaHQoKSk7XHJcbiAgICB0aGlzLnNldExvY2F0aW9uKHMuZ2V0TG9jYXRpb24oKS54LCBzLmdldExvY2F0aW9uKCkueSk7XHJcbiAgICB0aGlzLnNldE5leHQocy5nZXROZXh0KCkpO1xyXG4gICAgdGhpcy5zZXRPd25lcihzLmdldE93bmVyKCkpO1xyXG4gICAgdGhpcy5zZXRQcmVkMShzLmdldFByZWQxKCkpO1xyXG4gICAgdGhpcy5zZXRQcmVkMihzLmdldFByZWQyKCkpO1xyXG4gICAgdGhpcy5zZXRXaWR0aChzLmdldFdpZHRoKCkpO1xyXG5cclxuICAgIC8vIGNvcHkgZWRnZXNcclxuICAgIHZhciBudW1PZkVkZ2VzID0gcy5nZXRFZGdlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkVkZ2VzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSBzLmdldEVkZ2VzKClbaV07XHJcbiAgICAgICAgdmFyIG5ld0VkZ2UgPSBuZXcgU2JnblBERWRnZShlZGdlLmdldFNvdXJjZSgpLCBlZGdlLmdldFRhcmdldCgpLCBudWxsLCBlZGdlLnR5cGUpO1xyXG5cclxuICAgICAgICBuZXdFZGdlLmNvcHkoZWRnZSk7XHJcblxyXG4gICAgICAgIGlmIChlZGdlLmdldFNvdXJjZSgpID09PSBzKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgbmV3RWRnZS5zZXRTb3VyY2UodGhpcyk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKGVkZ2UuZ2V0VGFyZ2V0KCkgPT09IHMpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBuZXdFZGdlLnNldFRhcmdldCh0aGlzKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIC8vIGFkZCBuZXcgZWRnZSB0byB0aGUgZ3JhcGggbWFuYWdlci5cclxuICAgICAgICBncmFwaE1hbmFnZXIuYWRkKG5ld0VkZ2UsIG5ld0VkZ2UuZ2V0U291cmNlKCksIG5ld0VkZ2UuZ2V0VGFyZ2V0KCkpO1xyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5pc1ZlcnRpY2FsID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlRPUF9UT19CT1RUT00gfHwgXHJcbiAgICAgICAgdGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7ICAgXHJcbiAgICB9XHJcbiAgICBcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuaXNIb3Jpem9udGFsID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQgfHwgXHJcbiAgICAgICAgdGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uUklHSFRfVE9fTEVGVClcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLnNldE9yaWVudGF0aW9uID0gZnVuY3Rpb24gKG9yaWVudClcclxue1xyXG4gICAgdGhpcy5vcmllbnRhdGlvbiA9IG9yaWVudDtcclxuICAgIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuaW5wdXRQb3J0LnNldENlbnRlcih0aGlzLmdldENlbnRlclgoKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFX0xFTkdUSCwgdGhpcy5nZXRDZW50ZXJZKCkpO1xyXG4gICAgICAgIHRoaXMub3V0cHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKClcclxuICAgICAgICAgICAgICAgICAgICAgICAgKyBTYmduUERDb25zdGFudHMuUklHSURfRURHRV9MRU5HVEgsIHRoaXMuZ2V0Q2VudGVyWSgpKTtcclxuICAgIH1cclxuICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5pbnB1dFBvcnQuc2V0Q2VudGVyKHRoaXMuZ2V0Q2VudGVyWCgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RILCB0aGlzLmdldENlbnRlclkoKSk7XHJcbiAgICAgICAgdGhpcy5vdXRwdXRQb3J0LnNldENlbnRlcih0aGlzLmdldENlbnRlclgoKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFX0xFTkdUSCwgdGhpcy5nZXRDZW50ZXJZKCkpO1xyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgIHtcclxuICAgICAgICB0aGlzLmlucHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKCksIHRoaXMuZ2V0Q2VudGVyWSgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RIKTtcclxuICAgICAgICB0aGlzLm91dHB1dFBvcnQuc2V0Q2VudGVyKHRoaXMuZ2V0Q2VudGVyWCgpLCB0aGlzLmdldENlbnRlclkoKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFX0xFTkdUSCk7XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuaW5wdXRQb3J0LnNldENlbnRlcih0aGlzLmdldENlbnRlclgoKSwgdGhpcy5nZXRDZW50ZXJZKClcclxuICAgICAgICAgICAgICAgICAgICAgICAgLSBTYmduUERDb25zdGFudHMuUklHSURfRURHRV9MRU5HVEgpO1xyXG4gICAgICAgIHRoaXMub3V0cHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKCksIHRoaXMuZ2V0Q2VudGVyWSgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RIKTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRyYW5zZmVyIGZvcmNlcyBhY3Rpbmcgb24gcHJvY2VzcyBub2RlIHRvIGl0cyBwYXJlbnQgY29tcG91bmQgdG8gbWFrZVxyXG4qIHN1cmUgcHJvY2VzcyBkb2VzIG5vdCBtb3ZlLlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLnRyYW5zZmVyRm9yY2VzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdGhpcy5wYXJlbnRDb21wb3VuZC5zcHJpbmdGb3JjZVggKz0gdGhpcy5zcHJpbmdGb3JjZVhcclxuICAgICAgICAgICAgICAgICAgICArIHRoaXMuaW5wdXRQb3J0LnNwcmluZ0ZvcmNlWCArIHRoaXMub3V0cHV0UG9ydC5zcHJpbmdGb3JjZVg7XHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kLnNwcmluZ0ZvcmNlWSArPSB0aGlzLnNwcmluZ0ZvcmNlWVxyXG4gICAgICAgICAgICAgICAgICAgICsgdGhpcy5pbnB1dFBvcnQuc3ByaW5nRm9yY2VZICsgdGhpcy5vdXRwdXRQb3J0LnNwcmluZ0ZvcmNlWTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuZ2V0SW5wdXRQb3J0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMuaW5wdXRQb3J0O1xyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5nZXRPdXRwdXRQb3J0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMub3V0cHV0UG9ydDtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuZ2V0UGFyZW50Q29tcG91bmQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gcGFyZW50Q29tcG91bmQ7XHJcbn07XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IFNiZ25Qcm9jZXNzTm9kZTtcclxuIiwiZnVuY3Rpb24gVHJhbnNmb3JtKHgsIHkpIHtcbiAgdGhpcy5sd29ybGRPcmdYID0gMC4wO1xuICB0aGlzLmx3b3JsZE9yZ1kgPSAwLjA7XG4gIHRoaXMubGRldmljZU9yZ1ggPSAwLjA7XG4gIHRoaXMubGRldmljZU9yZ1kgPSAwLjA7XG4gIHRoaXMubHdvcmxkRXh0WCA9IDEuMDtcbiAgdGhpcy5sd29ybGRFeHRZID0gMS4wO1xuICB0aGlzLmxkZXZpY2VFeHRYID0gMS4wO1xuICB0aGlzLmxkZXZpY2VFeHRZID0gMS4wO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkT3JnWCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZE9yZ1g7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRPcmdYID0gZnVuY3Rpb24gKHdveClcbntcbiAgdGhpcy5sd29ybGRPcmdYID0gd294O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkT3JnWSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZE9yZ1k7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRPcmdZID0gZnVuY3Rpb24gKHdveSlcbntcbiAgdGhpcy5sd29ybGRPcmdZID0gd295O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkRXh0WCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZEV4dFg7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRFeHRYID0gZnVuY3Rpb24gKHdleClcbntcbiAgdGhpcy5sd29ybGRFeHRYID0gd2V4O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkRXh0WSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZEV4dFk7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRFeHRZID0gZnVuY3Rpb24gKHdleSlcbntcbiAgdGhpcy5sd29ybGRFeHRZID0gd2V5O1xufVxuXG4vKiBEZXZpY2UgcmVsYXRlZCAqL1xuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldERldmljZU9yZ1ggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sZGV2aWNlT3JnWDtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5zZXREZXZpY2VPcmdYID0gZnVuY3Rpb24gKGRveClcbntcbiAgdGhpcy5sZGV2aWNlT3JnWCA9IGRveDtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5nZXREZXZpY2VPcmdZID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMubGRldmljZU9yZ1k7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0RGV2aWNlT3JnWSA9IGZ1bmN0aW9uIChkb3kpXG57XG4gIHRoaXMubGRldmljZU9yZ1kgPSBkb3k7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuZ2V0RGV2aWNlRXh0WCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmxkZXZpY2VFeHRYO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLnNldERldmljZUV4dFggPSBmdW5jdGlvbiAoZGV4KVxue1xuICB0aGlzLmxkZXZpY2VFeHRYID0gZGV4O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldERldmljZUV4dFkgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sZGV2aWNlRXh0WTtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5zZXREZXZpY2VFeHRZID0gZnVuY3Rpb24gKGRleSlcbntcbiAgdGhpcy5sZGV2aWNlRXh0WSA9IGRleTtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS50cmFuc2Zvcm1YID0gZnVuY3Rpb24gKHgpXG57XG4gIHZhciB4RGV2aWNlID0gMC4wO1xuICB2YXIgd29ybGRFeHRYID0gdGhpcy5sd29ybGRFeHRYO1xuICBpZiAod29ybGRFeHRYICE9IDAuMClcbiAge1xuICAgIHhEZXZpY2UgPSB0aGlzLmxkZXZpY2VPcmdYICtcbiAgICAgICAgICAgICgoeCAtIHRoaXMubHdvcmxkT3JnWCkgKiB0aGlzLmxkZXZpY2VFeHRYIC8gd29ybGRFeHRYKTtcbiAgfVxuXG4gIHJldHVybiB4RGV2aWNlO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLnRyYW5zZm9ybVkgPSBmdW5jdGlvbiAoeSlcbntcbiAgdmFyIHlEZXZpY2UgPSAwLjA7XG4gIHZhciB3b3JsZEV4dFkgPSB0aGlzLmx3b3JsZEV4dFk7XG4gIGlmICh3b3JsZEV4dFkgIT0gMC4wKVxuICB7XG4gICAgeURldmljZSA9IHRoaXMubGRldmljZU9yZ1kgK1xuICAgICAgICAgICAgKCh5IC0gdGhpcy5sd29ybGRPcmdZKSAqIHRoaXMubGRldmljZUV4dFkgLyB3b3JsZEV4dFkpO1xuICB9XG5cblxuICByZXR1cm4geURldmljZTtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5pbnZlcnNlVHJhbnNmb3JtWCA9IGZ1bmN0aW9uICh4KVxue1xuICB2YXIgeFdvcmxkID0gMC4wO1xuICB2YXIgZGV2aWNlRXh0WCA9IHRoaXMubGRldmljZUV4dFg7XG4gIGlmIChkZXZpY2VFeHRYICE9IDAuMClcbiAge1xuICAgIHhXb3JsZCA9IHRoaXMubHdvcmxkT3JnWCArXG4gICAgICAgICAgICAoKHggLSB0aGlzLmxkZXZpY2VPcmdYKSAqIHRoaXMubHdvcmxkRXh0WCAvIGRldmljZUV4dFgpO1xuICB9XG5cblxuICByZXR1cm4geFdvcmxkO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmludmVyc2VUcmFuc2Zvcm1ZID0gZnVuY3Rpb24gKHkpXG57XG4gIHZhciB5V29ybGQgPSAwLjA7XG4gIHZhciBkZXZpY2VFeHRZID0gdGhpcy5sZGV2aWNlRXh0WTtcbiAgaWYgKGRldmljZUV4dFkgIT0gMC4wKVxuICB7XG4gICAgeVdvcmxkID0gdGhpcy5sd29ybGRPcmdZICtcbiAgICAgICAgICAgICgoeSAtIHRoaXMubGRldmljZU9yZ1kpICogdGhpcy5sd29ybGRFeHRZIC8gZGV2aWNlRXh0WSk7XG4gIH1cbiAgcmV0dXJuIHlXb3JsZDtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5pbnZlcnNlVHJhbnNmb3JtUG9pbnQgPSBmdW5jdGlvbiAoaW5Qb2ludClcbntcbiAgdmFyIG91dFBvaW50ID1cbiAgICAgICAgICBuZXcgUG9pbnREKHRoaXMuaW52ZXJzZVRyYW5zZm9ybVgoaW5Qb2ludC54KSxcbiAgICAgICAgICAgICAgICAgIHRoaXMuaW52ZXJzZVRyYW5zZm9ybVkoaW5Qb2ludC55KSk7XG4gIHJldHVybiBvdXRQb2ludDtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBUcmFuc2Zvcm07XG4iLCJmdW5jdGlvbiBVbmlxdWVJREdlbmVyZXRvcigpIHtcbn1cblxuVW5pcXVlSURHZW5lcmV0b3IubGFzdElEID0gMDtcblxuVW5pcXVlSURHZW5lcmV0b3IuY3JlYXRlSUQgPSBmdW5jdGlvbiAob2JqKSB7XG4gIGlmIChVbmlxdWVJREdlbmVyZXRvci5pc1ByaW1pdGl2ZShvYmopKSB7XG4gICAgcmV0dXJuIG9iajtcbiAgfVxuICBpZiAob2JqLnVuaXF1ZUlEICE9IG51bGwpIHtcbiAgICByZXR1cm4gb2JqLnVuaXF1ZUlEO1xuICB9XG4gIG9iai51bmlxdWVJRCA9IFVuaXF1ZUlER2VuZXJldG9yLmdldFN0cmluZygpO1xuICBVbmlxdWVJREdlbmVyZXRvci5sYXN0SUQrKztcbiAgcmV0dXJuIG9iai51bmlxdWVJRDtcbn1cblxuVW5pcXVlSURHZW5lcmV0b3IuZ2V0U3RyaW5nID0gZnVuY3Rpb24gKGlkKSB7XG4gIGlmIChpZCA9PSBudWxsKVxuICAgIGlkID0gVW5pcXVlSURHZW5lcmV0b3IubGFzdElEO1xuICByZXR1cm4gXCJPYmplY3QjXCIgKyBpZCArIFwiXCI7XG59XG5cblVuaXF1ZUlER2VuZXJldG9yLmlzUHJpbWl0aXZlID0gZnVuY3Rpb24gKGFyZykge1xuICB2YXIgdHlwZSA9IHR5cGVvZiBhcmc7XG4gIHJldHVybiBhcmcgPT0gbnVsbCB8fCAodHlwZSAhPSBcIm9iamVjdFwiICYmIHR5cGUgIT0gXCJmdW5jdGlvblwiKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBVbmlxdWVJREdlbmVyZXRvcjtcbiIsInZhciBDb1NFRWRnZSA9IHJlcXVpcmUoJy4vQ29TRUVkZ2UnKTtcclxuXHJcbmZ1bmN0aW9uIFZpc2liaWxpdHlFZGdlKHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSkge1xyXG4gIENvU0VFZGdlLmNhbGwodGhpcywgc291cmNlLCB0YXJnZXQsIHZFZGdlKTtcclxufVxyXG5cclxuVmlzaWJpbGl0eUVkZ2UucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShDb1NFRWRnZS5wcm90b3R5cGUpO1xyXG5mb3IgKHZhciBwcm9wIGluIENvU0VFZGdlKSB7XHJcbiAgVmlzaWJpbGl0eUVkZ2VbcHJvcF0gPSBDb1NFRWRnZVtwcm9wXTtcclxufVxyXG5cclxuLyoqXHJcbiogV2Ugd2FudCB0aGUgbGVuZ3RoIG9mIGEgdmlzaWJpbGl0eSBlZGdlIGNhbGN1bGF0ZWQgYXMgdGhlIGRpc3RhbmNlXHJcbiogYmV0d2VlbiBib3JkZXJzIG9mIHR3byBub2Rlcywgbm90IHRoZSBkaXN0YW5jZSBiZXR3ZWVuIGNlbnRlciBwb2ludHMuXHJcbiogRWRnZXMgaGF2ZSB0byBiZSB2ZXJ0aWNhbCBvciBob3Jpem9udGFsLlxyXG4qL1xyXG4vLyBAT3ZlcnJpZGVcclxuVmlzaWJpbGl0eUVkZ2UucHJvdG90eXBlLnVwZGF0ZUxlbmd0aCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIGlmICh0aGlzLnNvdXJjZS5nZXRCb3R0b20oKSA8PSB0aGlzLnRhcmdldC5nZXRUb3AoKSlcclxuICAgIHtcclxuICAgICAgICB0aGlzLmxlbmd0aFggPSAwO1xyXG4gICAgICAgIHRoaXMubGVuZ3RoWSA9IHRoaXMuc291cmNlLmdldEJvdHRvbSgpIC0gdGhpcy50YXJnZXQuZ2V0VG9wKCk7XHJcbiAgICB9IFxyXG4gICAgZWxzZSBpZiAodGhpcy5zb3VyY2UuZ2V0UmlnaHQoKSA8PSB0aGlzLnRhcmdldC5nZXRMZWZ0KCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5sZW5ndGhYID0gdGhpcy5zb3VyY2UuZ2V0UmlnaHQoKSAtIHRoaXMudGFyZ2V0LmdldExlZnQoKTtcclxuICAgICAgICB0aGlzLmxlbmd0aFkgPSAwO1xyXG4gICAgfVxyXG4gICAgLy8gZWxzZVxyXG4gICAgLy8gU3lzdGVtLm91dC5wcmludGxuKFwidW5leHBlY3RlZCBlZGdlXCIpO1xyXG5cclxuICAgIHRoaXMubGVuZ3RoID0gTWF0aC5zcXJ0KCh0aGlzLmxlbmd0aFggKiB0aGlzLmxlbmd0aFgpICsgKHRoaXMubGVuZ3RoWSAqIHRoaXMubGVuZ3RoWSkpO1xyXG59O1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBWaXNpYmlsaXR5RWRnZTtcclxuIiwidmFyIEludGVnZXIgPSByZXF1aXJlKCcuL0ludGVnZXInKTtcclxudmFyIFJlY3RhbmdsZUQgPSByZXF1aXJlKCcuL1JlY3RhbmdsZUQnKTtcclxuXHJcbnZhciBMR3JhcGggPSByZXF1aXJlKCcuL0xHcmFwaCcpO1xyXG52YXIgVmlzaWJpbGl0eUVkZ2UgPSByZXF1aXJlKCcuL1Zpc2liaWxpdHlFZGdlJyk7XHJcbnZhciBDb21wYWN0aW9uID0gcmVxdWlyZSgnLi9Db21wYWN0aW9uJyk7XHJcblxyXG5mdW5jdGlvbiBWaXNpYmlsaXR5R3JhcGgocGFyZW50LCBncmFwaE1nciwgdkdyYXBoKSBcclxue1xyXG4gICAgTEdyYXBoLmNhbGwodGhpcywgcGFyZW50LCBncmFwaE1nciwgdkdyYXBoKTtcclxuICAgIFxyXG4gICAgdGhpcy5kaXJlY3Rpb24gPSBudWxsO1xyXG59XHJcblxyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMR3JhcGgucHJvdG90eXBlKTtcclxuZm9yICh2YXIgcHJvcCBpbiBMR3JhcGgpIHtcclxuICBWaXNpYmlsaXR5R3JhcGhbcHJvcF0gPSBMR3JhcGhbcHJvcF07XHJcbn1cclxuXHJcbi8qKlxyXG4qIENyZWF0ZSBhIG5ldyB2aXNpYmlsaXR5IGdyYXBoLiBDb21wYXJlIGVhY2ggdmVydGljZXMgaW4gdGhlIGdyYXBoIHRvIHNlZVxyXG4qIGlmIHRoZXkgYXJlIHZpc2libGUgdG8gZWFjaCBvdGhlci4gSWYgdHdvIHZlcnRpY2VzIGNhbiBzZWUgZWFjaCBvdGhlciBhbmRcclxuKiB0aGV5IHNlZSBlYWNoIG90aGVyIGluIHRoZSBkZXNpcmVkIGRpcmVjdGlvbiwgYWRkIGFuIGVkZ2UgYmV0d2VlbiB0aGVtLlxyXG4qL1xyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlLmNvbnN0cnVjdCA9IGZ1bmN0aW9uIChkLCB2ZXJ0aWNlcylcclxue1xyXG4gICAgdGhpcy5pbml0KHZlcnRpY2VzKTtcclxuXHJcbiAgICB2YXIgbm9kZXMgPSB0aGlzLmdldE5vZGVzKCk7XHJcbiAgICB0aGlzLmRpcmVjdGlvbiA9IGQ7XHJcblxyXG4gICAgLy8gY2hlY2sgdGhlIHZpc2liaWxpdHkgYmV0d2VlbiBlYWNoIHR3byB2ZXJ0ZXhcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbm9kZXMubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgZm9yICh2YXIgaiA9IGkgKyAxOyBqIDwgbm9kZXMubGVuZ3RoOyBqKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgbm9kZTEgPSBub2Rlc1tpXTtcclxuICAgICAgICAgICAgdmFyIG5vZGUyID0gbm9kZXNbal07XHJcblxyXG4gICAgICAgICAgICB2YXIgcmVzdWx0ID0gdGhpcy5maW5kVmlzaWJpbGl0eURpcmVjdGlvbihub2RlMSwgbm9kZTIpO1xyXG5cclxuICAgICAgICAgICAgLy8gaWYgdGhleSBhcmUgdmlzaWJsZSwgY3JlYXRlIGVkZ2UuXHJcbiAgICAgICAgICAgIGlmIChyZXN1bHQgIT09IDApXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMuY3JlYXRlRWRnZShub2RlMSwgbm9kZTIpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgYWRkcyB0aGUgZ2l2ZW4gbm9kZXMgdG8gdGhlIGdyYXBoLlxyXG4qL1xyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlLmluaXQgPSBmdW5jdGlvbiAodmVydGljZXMpXHJcbntcclxuICAgIC8vIGNyZWF0ZSB0aGUgbmV3IGdyYXBoIHdpdGggZ2l2ZW4gdmVydGljZXNcclxuICAgIHZhciBudW1PZlZlcnRpY2VzID0gdmVydGljZXMubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaT0wOyBpPG51bU9mVmVydGljZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB0aGlzLmFkZCh2ZXJ0aWNlc1tpXSk7XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBHaXZlbiB0d28gbm9kZXMsIGNoZWNrIHRoZWlyIHZpc2liaWxpdHkuIFR3byBub2RlcyBhcmUgdmlzaWJsZSB0byBlYWNoXHJcbiogb3RoZXIgaWYgdGhlcmUgZXhpc3RzIGFuIGluZmluaXRlIHJheSB0aGF0IGludGVyc2VjdHMgdGhlbSB3aXRob3V0XHJcbiogaW50ZXJzZWN0aW5nIGFueSBvdGhlciBub2RlcyBpbiBiZXR3ZWVuIHRob3NlIHR3by5cclxuKiBcclxuKiBAcmV0dXJuIDE6IHZlcnRpY2FsLCAyOiBob3Jpem9udGFsIChpZiBhbnkgZWRnZSBmb3VuZCkuIE90aGVyd2lzZSwgcmV0dXJuXHJcbiogICAgICAgICAwXHJcbiovXHJcblZpc2liaWxpdHlHcmFwaC5wcm90b3R5cGUuZmluZFZpc2liaWxpdHlEaXJlY3Rpb24gPSBmdW5jdGlvbiAocCwgcSlcclxue1xyXG4gICAgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKVxyXG4gICAge1xyXG4gICAgICAgIC8vIGVuc3VyZSB0aGF0IHAgcG9pbnRzIHRvIHRoZSBsZWZ0bW9zdCBlbGVtZW50XHJcbiAgICAgICAgaWYgKHEuZ2V0TGVmdCgpIDwgcC5nZXRMZWZ0KCkgJiYgXHJcbiAgICAgICAgICAgIHAuZ2V0TGVmdCgpIDwgKHEuZ2V0TGVmdCgpICsgcS5nZXRXaWR0aCgpKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciB0ZW1wID0gcDtcclxuICAgICAgICAgICAgcCA9IHE7XHJcbiAgICAgICAgICAgIHEgPSB0ZW1wO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy8gY2hlY2sgaWYgdGhlcmUgZXhpc3RzIGEgcmF5XHJcbiAgICAgICAgaWYgKHAuZ2V0TGVmdCgpIDw9IHEuZ2V0TGVmdCgpICYmIFxyXG4gICAgICAgICAgICBxLmdldExlZnQoKSA8PSAocC5nZXRMZWZ0KCkgKyBwLmdldFdpZHRoKCkpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHRoaXMuc3dlZXBJbnRlcnNlY3RlZEFyZWEocCwgcSkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiAxO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgLy8gZW5zdXJlIHRoYXQgcCBwb2ludHMgdG8gdGhlIHVwcGVyIGVsZW1lbnRcclxuICAgICAgICBpZiAocS5nZXRUb3AoKSA8IHAuZ2V0VG9wKCkgJiYgXHJcbiAgICAgICAgICAgIHAuZ2V0VG9wKCkgPCAocS5nZXRUb3AoKSArIHEuZ2V0SGVpZ2h0KCkpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdmFyIHRlbXAgPSBwO1xyXG4gICAgICAgICAgICBwID0gcTtcclxuICAgICAgICAgICAgcSA9IHRlbXA7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBjaGVjayBpZiB0aGVyZSBleGlzdHMgYSByYXlcclxuICAgICAgICBpZiAocC5nZXRUb3AoKSA8PSBxLmdldFRvcCgpICYmIFxyXG4gICAgICAgICAgICBxLmdldFRvcCgpIDw9IChwLmdldFRvcCgpICsgcC5nZXRIZWlnaHQoKSkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodGhpcy5zd2VlcEludGVyc2VjdGVkQXJlYShwLCBxKSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDI7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICByZXR1cm4gMDtcclxufTtcclxuXHJcbi8qKlxyXG4qIFN0YXJ0aW5nIGZyb20gdGhlIGludGVyc2VjdGlvbiBhcmVhIGJldHdlZW4gcCBhbmQgcSwgd2FsayBvbiBhIGxpbmVcclxuKiBwZXJwZW5kaWN1bGFyIHRvIHRoZSBkZXNpcmVkIGRpcmVjdGlvbi4gSWYgdGhlcmUgaXMgYW4gZWRnZSB0aGF0IGRvZXMgbm90XHJcbiogaW50ZXJzZWN0IHdpdGggYW55IG90aGVyIG5vZGVzLCB0aGlzIGlzIGEgdmFsaWQgZWRnZS5cclxuKiBcclxuKiBAcmV0dXJuIHRydWUgaWYgYW4gZWRnZSBleGlzdHMuIGZhbHNlIG90aGVyd2lzZS5cclxuKi9cclxuVmlzaWJpbGl0eUdyYXBoLnByb3RvdHlwZS5zd2VlcEludGVyc2VjdGVkQXJlYSA9IGZ1bmN0aW9uIChwLCBxKVxyXG57XHJcbiAgICB2YXIgZWRnZTtcclxuICAgIHZhciBpc1ZhbGlkO1xyXG4gICAgdmFyIHN0YXJ0ID0gMDtcclxuICAgIHZhciBlbmQgPSAwO1xyXG4gICAgdmFyIHJlc3VsdDtcclxuXHJcbiAgICAvLyBmaW5kIHRoZSBzd2VlcCBsaW5lIGJvcmRlcnNcclxuICAgIGlmICh0aGlzLmRpcmVjdGlvbiA9PT0gQ29tcGFjdGlvbi5Db21wYWN0aW9uRGlyZWN0aW9uRW51bS5WRVJUSUNBTClcclxuICAgIHtcclxuICAgICAgICBzdGFydCA9IHEuZ2V0TGVmdCgpO1xyXG4gICAgICAgIGVuZCA9IE1hdGgubWluKHAuZ2V0UmlnaHQoKSwgcS5nZXRSaWdodCgpKTtcclxuICAgIH1cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgc3RhcnQgPSBxLmdldFRvcCgpO1xyXG4gICAgICAgIGVuZCA9IE1hdGgubWluKHAuZ2V0Qm90dG9tKCksIHEuZ2V0Qm90dG9tKCkpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGlmIHRoZXkgaW50ZXJzZWN0IG9ubHkgb24gdGhlIGJvcmRlcnMsIGltbWVkaWF0ZWx5IHJldHVybiBmYWxzZS5cclxuICAgIGlmIChzdGFydCA9PT0gZW5kKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBjaGVjayBmb3IgYWxsIGludGVyc2VjdGVkIGFyZWFcclxuICAgIGZvciAodmFyIHN3ZWVwUG9pbnQgPSBzdGFydDsgc3dlZXBQb2ludCA8PSBlbmQ7IHN3ZWVwUG9pbnQrKylcclxuICAgIHtcclxuICAgICAgICBpc1ZhbGlkID0gdHJ1ZTtcclxuICAgICAgICBlZGdlID0gdGhpcy50cnlDb25zdHJ1Y3RpbmdFZGdlKHAsIHEsIHN3ZWVwUG9pbnQpO1xyXG5cclxuICAgICAgICAvLyBpZiBhbiBlZGdlIGlzIGNvbnN0cnVjdGVkLCBjaGVjayBpdHMgdmFsaWRpdHlcclxuICAgICAgICBpZiAoZWRnZSAhPT0gbnVsbClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJlc3VsdCA9IHRoaXMuY2hlY2tJbnRlcm1lZGlhdGVOb2RlcyhwLCBxLCBlZGdlLCBzd2VlcFBvaW50KTtcclxuXHJcbiAgICAgICAgICAgIGlmIChzd2VlcFBvaW50ID09PSByZXN1bHQpXHJcbiAgICAgICAgICAgICAgICBpc1ZhbGlkID0gdHJ1ZTtcclxuICAgICAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBzd2VlcFBvaW50ID0gcmVzdWx0O1xyXG4gICAgICAgICAgICAgICAgaXNWYWxpZCA9IGZhbHNlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGlmIChpc1ZhbGlkKVxyXG4gICAgICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCB0cmllcyB0byBjb25zdHJ1Y3QgYW4gZWRnZShSZWN0YW5nbGVEIHNoYXBlKSBiZXR3ZWVuIHR3b1xyXG4qIG5vZGVzLiBUaGUgcGFyYW1ldGVyIGkgaW5kaWNhdGVzIHRoZSBzdGFydGluZyBwb2ludCBvZiB0aGUgZWRnZS4gRm9yXHJcbiogZXhhbXBsZSwgaWYgYSB2ZXJ0aWNhbCBlZGdlIHRvIGJlIGNvbnN0cnVjdGVkLCBpIGlzIHRoZSB0b3AgY29vcmRpbmF0ZS5cclxuKiBGb3IgaG9yaXpvbnRhbCwgaSBpcyB0aGUgbGVmdG1vc3QgY29vcmRpbmF0ZS5cclxuKiBcclxuKiBAcmV0dXJuIGVkZ2UuIGlmIG5vIGVkZ2UgY2FuIGJlIGNvbnN0cnVjdGVkLCByZXR1cm5zIG51bGwuXHJcbiovXHJcblZpc2liaWxpdHlHcmFwaC5wcm90b3R5cGUudHJ5Q29uc3RydWN0aW5nRWRnZSA9IGZ1bmN0aW9uIChwLCBxLCBpKVxyXG57XHJcbiAgICBpZiAodGhpcy5kaXJlY3Rpb24gPT09IENvbXBhY3Rpb24uQ29tcGFjdGlvbkRpcmVjdGlvbkVudW0uVkVSVElDQUwpXHJcbiAgICB7XHJcbiAgICAgICAgLy8gY3JlYXRlIGFuIGVkZ2UgZnJvbSB1cHBlciB0byBsb3dlciBvciByZXR1cm4gZmFsc2U6ZG9lcyBub3RcclxuICAgICAgICAvLyBleGlzdFxyXG4gICAgICAgIGlmIChwLmdldFRvcCgpIDwgcS5nZXRUb3AoKSAmJiBcclxuICAgICAgICAgICAgcC5nZXRCb3R0b20oKSA8PSBxLmdldFRvcCgpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBSZWN0YW5nbGVEKGksIHAuZ2V0Qm90dG9tKCksIDEsIChxLmdldFRvcCgpIC0gcC5nZXRCb3R0b20oKSkpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmIChxLmdldFRvcCgpIDwgcC5nZXRUb3AoKSAmJiBcclxuICAgICAgICAgICAgICAgICAocS5nZXRUb3AoKSArIHEuZ2V0SGVpZ2h0KCkpIDw9IHAuZ2V0VG9wKCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFJlY3RhbmdsZUQoaSwgcS5nZXRCb3R0b20oKSwgMSwgKHAuZ2V0VG9wKCkgLSBxLmdldEJvdHRvbSgpKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBudWxsO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgLy8gY3JlYXRlIGFuIGVkZ2UgZnJvbSBsZWZ0bW9zdCB0byByaWdodCBvciByZXR1cm4gZmFsc2U6ZG9lc1xyXG4gICAgICAgIC8vIG5vdCBleGlzdFxyXG4gICAgICAgIGlmIChwLmdldExlZnQoKSA8IHEuZ2V0TGVmdCgpICYmIFxyXG4gICAgICAgICAgICBwLmdldFJpZ2h0KCkgPD0gcS5nZXRMZWZ0KCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFJlY3RhbmdsZUQocC5nZXRSaWdodCgpLCBpLCAocS5nZXRMZWZ0KCkgLSBwLmdldFJpZ2h0KCkpLCAxKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAocS5nZXRMZWZ0KCkgPCBwLmdldExlZnQoKSAmJiBcclxuICAgICAgICAgICAgICAgICBxLmdldFJpZ2h0KCkgPD0gcC5nZXRMZWZ0KCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFJlY3RhbmdsZUQocS5nZXRSaWdodCgpLCBpLCAocC5nZXRMZWZ0KCkgLSBxLmdldFJpZ2h0KCkpLCAxKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG51bGw7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBudWxsO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY2hlY2tzIGlmIHRoZSBnaXZlbiBlZGdlIGludGVyc2VjdHMgYW55IG5vZGVzIGV4Y2VwdCB0aGVcclxuKiBzb3VyY2UgYW5kIHRhcmdldCBub2Rlcy4gSWYgYW4gaW50ZXJzZWN0aW9uIGlzIGZvdW5kLCB1cGRhdGUgdGhlIHN3ZWVwXHJcbiogcG9pbnQgdG8gdGhlIGVuZCBvZiB0aGUgaW50ZXJzZWN0ZWQgbm9kZS4gT3RoZXJ3aXNlLCBkbyBub3QgY2hhbmdlIHRoZVxyXG4qIHBvaW50LlxyXG4qL1xyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlLmNoZWNrSW50ZXJtZWRpYXRlTm9kZXMgPSBmdW5jdGlvbiAocCwgcSwgZWRnZSwgc3dlZXBQb2ludClcclxue1xyXG4gICAgZm9yICh2YXIgaiA9IDA7IGogPCB0aGlzLmdldE5vZGVzKCkubGVuZ3RoOyBqKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGludGVybWVkaWF0ZU5vZGUgPSB0aGlzLmdldE5vZGVzKClbal07XHJcblxyXG4gICAgICAgIGlmIChpbnRlcm1lZGlhdGVOb2RlICE9PSBwICYmIGludGVybWVkaWF0ZU5vZGUgIT09IHEpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICAvLyBpZiB0aGVyZSBpcyBhbiBpbnRlcnNlY3Rpb24sIGVkZ2UgaXMgbm90IHZhbGlkXHJcbiAgICAgICAgICAgIGlmIChlZGdlLmludGVyc2VjdHMoaW50ZXJtZWRpYXRlTm9kZS5nZXRSZWN0KCkpKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAvLyBqdW1wIHRvIHRoZSBlbmQgb2YgaW50ZXJzZWN0ZWQgbm9kZVxyXG4gICAgICAgICAgICAgICAgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKVxyXG4gICAgICAgICAgICAgICAgICAgIHN3ZWVwUG9pbnQgPSAoaW50ZXJtZWRpYXRlTm9kZS5nZXRSaWdodCgpICsgMSk7XHJcbiAgICAgICAgICAgICAgICBlbHNlIGlmICh0aGlzLmRpcmVjdGlvbiA9PT0gQ29tcGFjdGlvbi5Db21wYWN0aW9uRGlyZWN0aW9uRW51bS5IT1JJWk9OVEFMKVxyXG4gICAgICAgICAgICAgICAgICAgIHN3ZWVwUG9pbnQgPSAoaW50ZXJtZWRpYXRlTm9kZS5nZXRCb3R0b20oKSArIDEpO1xyXG5cclxuICAgICAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBzd2VlcFBvaW50O1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBjbGFzcyBjcmVhdGVzIGFuIGVkZ2UgYmV0d2VlbiB0aGUgZ2l2ZW4gbm9kZXMgdXNpbmcgdGhlIGdpdmVuXHJcbiogZGlyZWN0aW9uLiBXaGlsZSBhZGRpbmcgdGhlIGVkZ2UsIGJlIGNhcmVmdWwgYWJvdXQgdGhlIHNvdXJjZSBhbmQgdGFyZ2V0XHJcbiogaS5lLiBpZiB3ZSB3YW50IHRvIGdldCBhIHZlcnRpY2FsIHZpc2liaWxpdHkgZ3JhcGgsIChBIC0+IEIpIEEgc2hvdWxkXHJcbiogaGF2ZSBhIGxvd2VyIHkgY29vcmRpbmF0ZSAodXBwZXIpLiBTaW1pbGFybHksIGZvciBob3Jpem9udGFsIHZpc2liaWxpdHlcclxuKiBncmFwaCAoQSAtPiBCKTogQSBpcyBvbiB0aGUgbGVmdCwgaGFzIGxvd2VyIHggY29vcmRpbmF0ZS5cclxuKi9cclxuVmlzaWJpbGl0eUdyYXBoLnByb3RvdHlwZS5jcmVhdGVFZGdlID0gZnVuY3Rpb24gKG5vZGUxLCBub2RlMilcclxue1xyXG4gICAgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChub2RlMS5nZXRUb3AoKSA8IG5vZGUyLmdldFRvcCgpKVxyXG4gICAgICAgICAgICB0aGlzLmFkZChuZXcgVmlzaWJpbGl0eUVkZ2Uobm9kZTEsIG5vZGUyLCBudWxsKSwgbm9kZTEsIG5vZGUyKTtcclxuICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgIHRoaXMuYWRkKG5ldyBWaXNpYmlsaXR5RWRnZShub2RlMiwgbm9kZTEsIG51bGwpLCBub2RlMiwgbm9kZTEpO1xyXG4gICAgfVxyXG5cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKG5vZGUxLmdldExlZnQoKSA8IG5vZGUyLmdldExlZnQoKSlcclxuICAgICAgICAgICAgdGhpcy5hZGQobmV3IFZpc2liaWxpdHlFZGdlKG5vZGUxLCBub2RlMiwgbnVsbCksIG5vZGUxLCBub2RlMik7XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICB0aGlzLmFkZChuZXcgVmlzaWJpbGl0eUVkZ2Uobm9kZTIsIG5vZGUxLCBudWxsKSwgbm9kZTIsIG5vZGUxKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBjYWxjdWxhdGUgbmV3bHkgYWRkZWQgZWRnZSdzIGxlbmd0aC5cclxuICAgIHRoaXMuZ2V0RWRnZXMoKVt0aGlzLmdldEVkZ2VzKCkubGVuZ3RoIC0gMV0udXBkYXRlTGVuZ3RoKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBGb3IgZWFjaCBlZGdlIGhhdmluZyBzIGFzIGl0cyB0YXJnZXQgbm9kZSwgZmluZCBhbmQgcmV0dXJuIHRoZSBzaG9ydGVzdFxyXG4qIG9uZS4gUmV0dXJucyBudWxsIGlmIGNvdWxkIG5vdCBmaW5kIGFuIGVkZ2UuXHJcbiovXHJcblZpc2liaWxpdHlHcmFwaC5wcm90b3R5cGUuZmluZFNob3J0ZXN0RWRnZSA9IGZ1bmN0aW9uIChzKVxyXG57XHJcbiAgICB2YXIgc2hvcnRlc3RFZGdlID0gbnVsbDtcclxuICAgIHZhciBtaW5MZW5ndGggPSBJbnRlZ2VyLk1BWF9WQUxVRTtcclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMuZ2V0RWRnZXMoKS5sZW5ndGg7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZSA9IHRoaXMuZ2V0RWRnZXMoKVtpXTtcclxuXHJcbiAgICAgICAgZS51cGRhdGVMZW5ndGgoKTtcclxuICAgICAgICBpZiAoZS5nZXRUYXJnZXQoKSA9PT0gcyAmJiBlLmdldExlbmd0aCgpIDwgbWluTGVuZ3RoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgc2hvcnRlc3RFZGdlID0gZTtcclxuICAgICAgICAgICAgbWluTGVuZ3RoID0gZS5nZXRMZW5ndGgoKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIHNob3J0ZXN0RWRnZTtcclxufTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gVmlzaWJpbGl0eUdyYXBoO1xyXG4iLCIndXNlIHN0cmljdCc7XHJcblxyXG52YXIgVGhyZWFkO1xyXG5cclxudmFyIERpbWVuc2lvbkQgPSByZXF1aXJlKCcuL0RpbWVuc2lvbkQnKTtcclxudmFyIEhhc2hNYXAgPSByZXF1aXJlKCcuL0hhc2hNYXAnKTtcclxudmFyIEhhc2hTZXQgPSByZXF1aXJlKCcuL0hhc2hTZXQnKTtcclxudmFyIElHZW9tZXRyeSA9IHJlcXVpcmUoJy4vSUdlb21ldHJ5Jyk7XHJcbnZhciBJTWF0aCA9IHJlcXVpcmUoJy4vSU1hdGgnKTtcclxudmFyIEludGVnZXIgPSByZXF1aXJlKCcuL0ludGVnZXInKTtcclxudmFyIFBvaW50ID0gcmVxdWlyZSgnLi9Qb2ludCcpO1xyXG52YXIgUG9pbnREID0gcmVxdWlyZSgnLi9Qb2ludEQnKTtcclxudmFyIFJhbmRvbVNlZWQgPSByZXF1aXJlKCcuL1JhbmRvbVNlZWQnKTtcclxudmFyIFJlY3RhbmdsZUQgPSByZXF1aXJlKCcuL1JlY3RhbmdsZUQnKTtcclxudmFyIFRyYW5zZm9ybSA9IHJlcXVpcmUoJy4vVHJhbnNmb3JtJyk7XHJcbnZhciBVbmlxdWVJREdlbmVyZXRvciA9IHJlcXVpcmUoJy4vVW5pcXVlSURHZW5lcmV0b3InKTtcclxudmFyIExHcmFwaE9iamVjdCA9IHJlcXVpcmUoJy4vTEdyYXBoT2JqZWN0Jyk7XHJcbnZhciBMR3JhcGggPSByZXF1aXJlKCcuL0xHcmFwaCcpO1xyXG52YXIgTEVkZ2UgPSByZXF1aXJlKCcuL0xFZGdlJyk7XHJcbnZhciBMR3JhcGhNYW5hZ2VyID0gcmVxdWlyZSgnLi9MR3JhcGhNYW5hZ2VyJyk7XHJcbnZhciBMTm9kZSA9IHJlcXVpcmUoJy4vTE5vZGUnKTtcclxudmFyIExheW91dCA9IHJlcXVpcmUoJy4vTGF5b3V0Jyk7XHJcbnZhciBMYXlvdXRDb25zdGFudHMgPSByZXF1aXJlKCcuL0xheW91dENvbnN0YW50cycpO1xyXG52YXIgRkRMYXlvdXQgPSByZXF1aXJlKCcuL0ZETGF5b3V0Jyk7XHJcbnZhciBGRExheW91dENvbnN0YW50cyA9IHJlcXVpcmUoJy4vRkRMYXlvdXRDb25zdGFudHMnKTtcclxudmFyIEZETGF5b3V0RWRnZSA9IHJlcXVpcmUoJy4vRkRMYXlvdXRFZGdlJyk7XHJcbnZhciBGRExheW91dE5vZGUgPSByZXF1aXJlKCcuL0ZETGF5b3V0Tm9kZScpO1xyXG52YXIgQ29TRUNvbnN0YW50cyA9IHJlcXVpcmUoJy4vQ29TRUNvbnN0YW50cycpO1xyXG52YXIgQ29TRUVkZ2UgPSByZXF1aXJlKCcuL0NvU0VFZGdlJyk7XHJcbnZhciBDb1NFR3JhcGggPSByZXF1aXJlKCcuL0NvU0VHcmFwaCcpO1xyXG52YXIgQ29TRUdyYXBoTWFuYWdlciA9IHJlcXVpcmUoJy4vQ29TRUdyYXBoTWFuYWdlcicpO1xyXG52YXIgQ29TRUxheW91dCA9IHJlcXVpcmUoJy4vQ29TRUxheW91dCcpO1xyXG52YXIgQ29TRU5vZGUgPSByZXF1aXJlKCcuL0NvU0VOb2RlJyk7XHJcbnZhciBDb21wYWN0aW9uID0gcmVxdWlyZSgnLi9Db21wYWN0aW9uJyk7XHJcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xyXG52YXIgU2JnblBERWRnZSA9IHJlcXVpcmUoJy4vU2JnblBERWRnZScpO1xyXG52YXIgU2JnblBETGF5b3V0ID0gcmVxdWlyZSgnLi9TYmduUERMYXlvdXQnKTtcclxudmFyIFNiZ25QRE5vZGUgPSByZXF1aXJlKCcuL1NiZ25QRE5vZGUnKTtcclxudmFyIFNiZ25Qcm9jZXNzTm9kZSA9IHJlcXVpcmUoJy4vU2JnblByb2Nlc3NOb2RlJyk7XHJcbnZhciBWaXNpYmlsaXR5RWRnZSA9IHJlcXVpcmUoJy4vVmlzaWJpbGl0eUVkZ2UnKTtcclxudmFyIFZpc2liaWxpdHlHcmFwaCA9IHJlcXVpcmUoJy4vVmlzaWJpbGl0eUdyYXBoJyk7XHJcbnZhciBNZW1iZXJQYWNrID0gcmVxdWlyZSgnLi9NZW1iZXJQYWNrJyk7XHJcbnZhciBPcmdhbml6YXRpb24gPSByZXF1aXJlKCcuL09yZ2FuaXphdGlvbicpO1xyXG52YXIgUG9seW9taW5vUXVpY2tTb3J0ID0gcmVxdWlyZSgnLi9Qb2x5b21pbm9RdWlja1NvcnQnKTtcclxudmFyIFBvbHlvbWlub1BhY2tpbmcgPSByZXF1aXJlKCcuL1BvbHlvbWlub1BhY2tpbmcnKTtcclxudmFyIFJlY3RQcm9jID0gcmVxdWlyZSgnLi9SZWN0UHJvYycpO1xyXG5cclxuX1NiZ25QRExheW91dC5pZFRvTE5vZGUgPSB7fTtcclxuX1NiZ25QRExheW91dC50b0JlVGlsZWQgPSB7fTtcclxuXHJcbi8vIFRPRE86IEJ1bmxhciBsYXlvdXQgZGVmYXVsdCdsYXJpIG1pPyBZb2tzYSBDb1NFIHNwZWNpZmljIGRlZmF1bHQnbGFyIG1pP1xyXG5cclxudmFyIGRlZmF1bHRzID0ge1xyXG4gICAgLy8gQ2FsbGVkIG9uIGBsYXlvdXRyZWFkeWBcclxuICAgIHJlYWR5OiBmdW5jdGlvbiAoKSB7XHJcbiAgICB9LFxyXG4gICAgLy8gQ2FsbGVkIG9uIGBsYXlvdXRzdG9wYFxyXG4gICAgc3RvcDogZnVuY3Rpb24gKCkge1xyXG4gICAgfSxcclxuICAgIC8vIFdoZXRoZXIgdG8gZml0IHRoZSBuZXR3b3JrIHZpZXcgYWZ0ZXIgd2hlbiBkb25lXHJcbiAgICBmaXQ6IHRydWUsXHJcbiAgICAvLyBQYWRkaW5nIG9uIGZpdFxyXG4gICAgcGFkZGluZzogMTAsXHJcbiAgICAvLyBXaGV0aGVyIHRvIGVuYWJsZSBpbmNyZW1lbnRhbCBtb2RlXHJcbiAgICByYW5kb21pemU6IHRydWUsXHJcbiAgICAvLyBOb2RlIHJlcHVsc2lvbiAobm9uIG92ZXJsYXBwaW5nKSBtdWx0aXBsaWVyXHJcbiAgICBub2RlUmVwdWxzaW9uOiA0NTAwLFxyXG4gICAgLy8gSWRlYWwgZWRnZSAobm9uIG5lc3RlZCkgbGVuZ3RoXHJcbiAgICBpZGVhbEVkZ2VMZW5ndGg6IDUwLFxyXG4gICAgLy8gRGl2aXNvciB0byBjb21wdXRlIGVkZ2UgZm9yY2VzXHJcbiAgICBlZGdlRWxhc3RpY2l0eTogMC40NSxcclxuICAgIC8vIE5lc3RpbmcgZmFjdG9yIChtdWx0aXBsaWVyKSB0byBjb21wdXRlIGlkZWFsIGVkZ2UgbGVuZ3RoIGZvciBuZXN0ZWQgZWRnZXNcclxuICAgIG5lc3RpbmdGYWN0b3I6IDAuMSxcclxuICAgIC8vIEdyYXZpdHkgZm9yY2UgKGNvbnN0YW50KVxyXG4gICAgZ3Jhdml0eTogMC4yNSxcclxuICAgIC8vIE1heGltdW0gbnVtYmVyIG9mIGl0ZXJhdGlvbnMgdG8gcGVyZm9ybVxyXG4gICAgbnVtSXRlcjogMjUwMCxcclxuICAgIC8vIEZvciBlbmFibGluZyB0aWxpbmdcclxuICAgIHRpbGU6IHRydWUsXHJcbiAgICAvLyBUeXBlIG9mIGxheW91dCBhbmltYXRpb24uIFRoZSBvcHRpb24gc2V0IGlzIHsnZHVyaW5nJywgJ2VuZCcsIGZhbHNlfVxyXG4gICAgYW5pbWF0ZTogJ2VuZCcsXHJcbiAgICAvLyBSZXByZXNlbnRzIHRoZSBhbW91bnQgb2YgdGhlIHZlcnRpY2FsIHNwYWNlIHRvIHB1dCBiZXR3ZWVuIHRoZSB6ZXJvIGRlZ3JlZSBtZW1iZXJzIGR1cmluZyB0aGUgdGlsaW5nIG9wZXJhdGlvbihjYW4gYWxzbyBiZSBhIGZ1bmN0aW9uKVxyXG4gICAgdGlsaW5nUGFkZGluZ1ZlcnRpY2FsOiAxMCxcclxuICAgIC8vIFJlcHJlc2VudHMgdGhlIGFtb3VudCBvZiB0aGUgaG9yaXpvbnRhbCBzcGFjZSB0byBwdXQgYmV0d2VlbiB0aGUgemVybyBkZWdyZWUgbWVtYmVycyBkdXJpbmcgdGhlIHRpbGluZyBvcGVyYXRpb24oY2FuIGFsc28gYmUgYSBmdW5jdGlvbilcclxuICAgIHRpbGluZ1BhZGRpbmdIb3Jpem9udGFsOiAxMCxcclxuICAgIC8vIEdyYXZpdHkgcmFuZ2UgKGNvbnN0YW50KSBmb3IgY29tcG91bmRzXHJcbiAgICBncmF2aXR5UmFuZ2VDb21wb3VuZDogMS41LFxyXG4gICAgLy8gR3Jhdml0eSBmb3JjZSAoY29uc3RhbnQpIGZvciBjb21wb3VuZHNcclxuICAgIGdyYXZpdHlDb21wb3VuZDogMS4wLFxyXG4gICAgLy8gR3Jhdml0eSByYW5nZSAoY29uc3RhbnQpXHJcbiAgICBncmF2aXR5UmFuZ2U6IDMuOFxyXG59O1xyXG5cclxuZnVuY3Rpb24gZXh0ZW5kKGRlZmF1bHRzLCBvcHRpb25zKSB7XHJcbiAgICB2YXIgb2JqID0ge307XHJcblxyXG4gICAgZm9yICh2YXIgaSBpbiBkZWZhdWx0cykge1xyXG4gICAgICAgIG9ialtpXSA9IGRlZmF1bHRzW2ldO1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIGkgaW4gb3B0aW9ucykge1xyXG4gICAgICAgIG9ialtpXSA9IG9wdGlvbnNbaV07XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIG9iajtcclxufVxyXG47XHJcblxyXG5fU2JnblBETGF5b3V0LmxheW91dCA9IG5ldyBTYmduUERMYXlvdXQoKTtcclxuZnVuY3Rpb24gX1NiZ25QRExheW91dChvcHRpb25zKSB7XHJcblxyXG4gICAgdGhpcy5vcHRpb25zID0gZXh0ZW5kKGRlZmF1bHRzLCBvcHRpb25zKTtcclxuICAgIF9TYmduUERMYXlvdXQuZ2V0VXNlck9wdGlvbnModGhpcy5vcHRpb25zKTtcclxufVxyXG5cclxuX1NiZ25QRExheW91dC5nZXRVc2VyT3B0aW9ucyA9IGZ1bmN0aW9uIChvcHRpb25zKSBcclxue1xyXG4gICAgLyoqIFRPRE86IERvIHdlIG5lZWQgbW9yZSBjb25zdGFuc3RzIChTQkdOIHNwZWNpZmljKSBoZXJlPyAqL1xyXG4gICAgXHJcbiAgICBpZiAob3B0aW9ucy5ub2RlUmVwdWxzaW9uICE9IG51bGwpXHJcbiAgICAgICAgU2JnblBEQ29uc3RhbnRzLkRFRkFVTFRfUkVQVUxTSU9OX1NUUkVOR1RIID0gQ29TRUNvbnN0YW50cy5ERUZBVUxUX1JFUFVMU0lPTl9TVFJFTkdUSCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfUkVQVUxTSU9OX1NUUkVOR1RIID0gb3B0aW9ucy5ub2RlUmVwdWxzaW9uO1xyXG4gICAgaWYgKG9wdGlvbnMuaWRlYWxFZGdlTGVuZ3RoICE9IG51bGwpXHJcbiAgICAgICAgU2JnblBEQ29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEggPSBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEggPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIID0gb3B0aW9ucy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICBpZiAob3B0aW9ucy5lZGdlRWxhc3RpY2l0eSAhPSBudWxsKVxyXG4gICAgICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX1NQUklOR19TVFJFTkdUSCA9IENvU0VDb25zdGFudHMuREVGQVVMVF9TUFJJTkdfU1RSRU5HVEggPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX1NQUklOR19TVFJFTkdUSCA9IG9wdGlvbnMuZWRnZUVsYXN0aWNpdHk7XHJcbiAgICBpZiAob3B0aW9ucy5uZXN0aW5nRmFjdG9yICE9IG51bGwpXHJcbiAgICAgICAgU2JnblBEQ29uc3RhbnRzLlBFUl9MRVZFTF9JREVBTF9FREdFX0xFTkdUSF9GQUNUT1IgPSBDb1NFQ29uc3RhbnRzLlBFUl9MRVZFTF9JREVBTF9FREdFX0xFTkdUSF9GQUNUT1IgPSBGRExheW91dENvbnN0YW50cy5QRVJfTEVWRUxfSURFQUxfRURHRV9MRU5HVEhfRkFDVE9SID0gb3B0aW9ucy5uZXN0aW5nRmFjdG9yO1xyXG4gICAgaWYgKG9wdGlvbnMuZ3Jhdml0eSAhPSBudWxsKVxyXG4gICAgICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX0dSQVZJVFlfU1RSRU5HVEggPSBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9TVFJFTkdUSCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9TVFJFTkdUSCA9IG9wdGlvbnMuZ3Jhdml0eTtcclxuICAgIGlmIChvcHRpb25zLm51bUl0ZXIgIT0gbnVsbClcclxuICAgICAgICBTYmduUERDb25zdGFudHMuTUFYX0lURVJBVElPTlMgPSBDb1NFQ29uc3RhbnRzLk1BWF9JVEVSQVRJT05TID0gRkRMYXlvdXRDb25zdGFudHMuTUFYX0lURVJBVElPTlMgPSBvcHRpb25zLm51bUl0ZXI7XHJcbiAgICBpZiAob3B0aW9ucy5ncmF2aXR5UmFuZ2UgIT0gbnVsbClcclxuICAgICAgICBTYmduUERDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IENvU0VDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9SQU5HRV9GQUNUT1IgPSBvcHRpb25zLmdyYXZpdHlSYW5nZTtcclxuICAgIGlmIChvcHRpb25zLmdyYXZpdHlDb21wb3VuZCAhPSBudWxsKVxyXG4gICAgICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX0NPTVBPVU5EX0dSQVZJVFlfU1RSRU5HVEggPSBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9TVFJFTkdUSCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9TVFJFTkdUSCA9IG9wdGlvbnMuZ3Jhdml0eUNvbXBvdW5kO1xyXG4gICAgaWYgKG9wdGlvbnMuZ3Jhdml0eVJhbmdlQ29tcG91bmQgIT0gbnVsbClcclxuICAgICAgICBTYmduUERDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IENvU0VDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9SQU5HRV9GQUNUT1IgPSBvcHRpb25zLmdyYXZpdHlSYW5nZUNvbXBvdW5kO1xyXG5cclxuICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID0gQ29TRUNvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9JTkNSRU1FTlRBTCA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID1cclxuICAgICAgICAgICAgIShvcHRpb25zLnJhbmRvbWl6ZSk7XHJcbiAgICBTYmduUERDb25zdGFudHMuQU5JTUFURSA9IENvU0VDb25zdGFudHMuQU5JTUFURSA9IEZETGF5b3V0Q29uc3RhbnRzLkFOSU1BVEUgPSBvcHRpb25zLmFuaW1hdGU7XHJcbn07XHJcblxyXG5fU2JnblBETGF5b3V0LnByb3RvdHlwZS5ydW4gPSBmdW5jdGlvbiAoKSB7XHJcbiAgICB2YXIgbGF5b3V0ID0gdGhpcztcclxuXHJcbiAgICBfU2JnblBETGF5b3V0LmlkVG9MTm9kZSA9IHt9O1xyXG4gICAgX1NiZ25QRExheW91dC50b0JlVGlsZWQgPSB7fTtcclxuICAgIF9TYmduUERMYXlvdXQubGF5b3V0ID0gbmV3IFNiZ25QRExheW91dCgpO1xyXG4gICAgdGhpcy5jeSA9IHRoaXMub3B0aW9ucy5jeTtcclxuICAgIHZhciBhZnRlciA9IHRoaXM7XHJcblxyXG4gICAgdGhpcy5jeS50cmlnZ2VyKCdsYXlvdXRzdGFydCcpO1xyXG5cclxuICAgIHZhciBnbSA9IF9TYmduUERMYXlvdXQubGF5b3V0Lm5ld0dyYXBoTWFuYWdlcigpO1xyXG4gICAgdGhpcy5nbSA9IGdtO1xyXG5cclxuICAgIHZhciBub2RlcyA9IHRoaXMub3B0aW9ucy5lbGVzLm5vZGVzKCk7XHJcbiAgICB2YXIgZWRnZXMgPSB0aGlzLm9wdGlvbnMuZWxlcy5lZGdlcygpO1xyXG5cclxuICAgIHRoaXMucm9vdCA9IGdtLmFkZFJvb3QoKTtcclxuXHJcbi8vICAgIGlmICghdGhpcy5vcHRpb25zLnRpbGUpIHtcclxuICAgICAgICB0aGlzLnByb2Nlc3NDaGlsZHJlbkxpc3QodGhpcy5yb290LCBfU2JnblBETGF5b3V0LmdldFRvcE1vc3ROb2Rlcyhub2RlcykpO1xyXG4vLyAgICB9IGVsc2Uge1xyXG4vLyAgICAgICAgLy8gRmluZCB6ZXJvIGRlZ3JlZSBub2RlcyBhbmQgY3JlYXRlIGEgY29tcG91bmQgZm9yIGVhY2ggbGV2ZWxcclxuLy8gICAgICAgIHZhciBtZW1iZXJHcm91cHMgPSB0aGlzLmdyb3VwWmVyb0RlZ3JlZU1lbWJlcnMoKTtcclxuLy8gICAgICAgIC8vIFRpbGUgYW5kIGNsZWFyIGNoaWxkcmVuIG9mIGVhY2ggY29tcG91bmRcclxuLy8gICAgICAgIHZhciB0aWxlZE1lbWJlclBhY2sgPSB0aGlzLmNsZWFyQ29tcG91bmRzKHRoaXMub3B0aW9ucyk7XHJcbi8vICAgICAgICAvLyBTZXBhcmF0ZWx5IHRpbGUgYW5kIGNsZWFyIHplcm8gZGVncmVlIG5vZGVzIGZvciBlYWNoIGxldmVsXHJcbi8vICAgICAgICB2YXIgdGlsZWRaZXJvRGVncmVlTm9kZXMgPSB0aGlzLmNsZWFyWmVyb0RlZ3JlZU1lbWJlcnMobWVtYmVyR3JvdXBzKTtcclxuLy8gICAgfVxyXG5cclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IGVkZ2VzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSBlZGdlc1tpXTtcclxuICAgICAgICB2YXIgc291cmNlTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2VkZ2UuZGF0YShcInNvdXJjZVwiKV07XHJcbiAgICAgICAgdmFyIHRhcmdldE5vZGUgPSBfU2JnblBETGF5b3V0LmlkVG9MTm9kZVtlZGdlLmRhdGEoXCJ0YXJnZXRcIildO1xyXG4gICAgICAgIHZhciBlMSA9IGdtLmFkZChfU2JnblBETGF5b3V0LmxheW91dC5uZXdFZGdlKCksIHNvdXJjZU5vZGUsIHRhcmdldE5vZGUpO1xyXG4gICAgICAgIGUxLmlkID0gZWRnZS5pZCgpO1xyXG4gICAgfVxyXG5cclxuXHJcbiAgICB2YXIgdDEgPSBsYXlvdXQudGhyZWFkO1xyXG5cclxuICAgIGlmICghdDEgfHwgdDEuc3RvcHBlZCgpKSB7IC8vIHRyeSB0byByZXVzZSB0aHJlYWRzXHJcbiAgICAgICAgdDEgPSBsYXlvdXQudGhyZWFkID0gVGhyZWFkKCk7XHJcblxyXG4gICAgICAgIHQxLnJlcXVpcmUoRGltZW5zaW9uRCwgJ0RpbWVuc2lvbkQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKEhhc2hNYXAsICdIYXNoTWFwJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShIYXNoU2V0LCAnSGFzaFNldCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoSUdlb21ldHJ5LCAnSUdlb21ldHJ5Jyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShJTWF0aCwgJ0lNYXRoJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShJbnRlZ2VyLCAnSW50ZWdlcicpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoUG9pbnQsICdQb2ludCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoUG9pbnRELCAnUG9pbnREJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShSYW5kb21TZWVkLCAnUmFuZG9tU2VlZCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoUmVjdGFuZ2xlRCwgJ1JlY3RhbmdsZUQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFRyYW5zZm9ybSwgJ1RyYW5zZm9ybScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoVW5pcXVlSURHZW5lcmV0b3IsICdVbmlxdWVJREdlbmVyZXRvcicpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoTEdyYXBoT2JqZWN0LCAnTEdyYXBoT2JqZWN0Jyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShMR3JhcGgsICdMR3JhcGgnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKExFZGdlLCAnTEVkZ2UnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKExHcmFwaE1hbmFnZXIsICdMR3JhcGhNYW5hZ2VyJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShMTm9kZSwgJ0xOb2RlJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShMYXlvdXQsICdMYXlvdXQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKExheW91dENvbnN0YW50cywgJ0xheW91dENvbnN0YW50cycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoRkRMYXlvdXQsICdGRExheW91dCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoRkRMYXlvdXRDb25zdGFudHMsICdGRExheW91dENvbnN0YW50cycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoRkRMYXlvdXRFZGdlLCAnRkRMYXlvdXRFZGdlJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShGRExheW91dE5vZGUsICdGRExheW91dE5vZGUnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKENvU0VDb25zdGFudHMsICdDb1NFQ29uc3RhbnRzJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShDb1NFRWRnZSwgJ0NvU0VFZGdlJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShDb1NFR3JhcGgsICdDb1NFR3JhcGgnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKENvU0VHcmFwaE1hbmFnZXIsICdDb1NFR3JhcGhNYW5hZ2VyJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShDb1NFTGF5b3V0LCAnQ29TRUxheW91dCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoQ29TRU5vZGUsICdDb1NFTm9kZScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoQ29tcGFjdGlvbiwgJ0NvbXBhY3Rpb24nKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFNiZ25QRENvbnN0YW50cywgJ1NiZ25QRENvbnN0YW50cycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoU2JnblBERWRnZSwgJ1NiZ25QREVkZ2UnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFNiZ25QRExheW91dCwgJ1NiZ25QRExheW91dCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoU2JnblBETm9kZSwgJ1NiZ25QRE5vZGUnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFNiZ25Qcm9jZXNzTm9kZSwgJ1NiZ25Qcm9jZXNzTm9kZScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoVmlzaWJpbGl0eUVkZ2UsICdWaXNpYmlsaXR5RWRnZScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoVmlzaWJpbGl0eUdyYXBoLCAnVmlzaWJpbGl0eUdyYXBoJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShNZW1iZXJQYWNrLCAnTWVtYmVyUGFjaycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoT3JnYW5pemF0aW9uLCAnT3JnYW5pemF0aW9uJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShQb2x5b21pbm9RdWlja1NvcnQsICdQb2x5b21pbm9RdWlja1NvcnQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFBvbHlvbWlub1BhY2tpbmcsICdQb2x5b21pbm9QYWNraW5nJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShSZWN0UHJvYywgJ1JlY3RQcm9jJyk7XHJcbiAgICB9XHJcblxyXG4gICAgdmFyIG5vZGVzID0gdGhpcy5vcHRpb25zLmVsZXMubm9kZXMoKTtcclxuICAgIHZhciBlZGdlcyA9IHRoaXMub3B0aW9ucy5lbGVzLmVkZ2VzKCk7XHJcblxyXG4gICAgLy8gRmlyc3QgSSBuZWVkIHRvIGNyZWF0ZSB0aGUgZGF0YSBzdHJ1Y3R1cmUgdG8gcGFzcyB0byB0aGUgd29ya2VyXHJcbiAgICB2YXIgcERhdGEgPSB7XHJcbiAgICAgICAgJ25vZGVzJzogW10sXHJcbiAgICAgICAgJ2VkZ2VzJzogW11cclxuICAgIH07XHJcblxyXG4gICAgLy9NYXAgdGhlIGlkcyBvZiBub2RlcyBpbiB0aGUgbGlzdCB0byBjaGVjayBpZiBhIG5vZGUgaXMgaW4gdGhlIGxpc3QgaW4gY29uc3RhbnQgdGltZVxyXG4gICAgdmFyIG5vZGVJZE1hcCA9IHt9O1xyXG5cclxuICAgIC8vRmlsbCB0aGUgbWFwIGluIGxpbmVhciB0aW1lXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IG5vZGVzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgbm9kZUlkTWFwW25vZGVzW2ldLmlkKCldID0gdHJ1ZTtcclxuICAgIH1cclxuXHJcbiAgICB2YXIgbG5vZGVzID0gZ20uZ2V0QWxsTm9kZXMoKTtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbG5vZGVzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgdmFyIGxub2RlID0gbG5vZGVzW2ldO1xyXG4gICAgICAgIHZhciBub2RlSWQgPSBsbm9kZS5pZDtcclxuICAgICAgICB2YXIgY3lOb2RlID0gdGhpcy5vcHRpb25zLmN5LmdldEVsZW1lbnRCeUlkKG5vZGVJZCk7XHJcblxyXG4gICAgICAgIHZhciBwYXJlbnRJZCA9IGN5Tm9kZS5kYXRhKCdwYXJlbnQnKTtcclxuICAgICAgICBwYXJlbnRJZCA9IG5vZGVJZE1hcFtwYXJlbnRJZF0gPyBwYXJlbnRJZCA6IHVuZGVmaW5lZDtcclxuXHJcbiAgICAgICAgdmFyIHcgPSBsbm9kZS5yZWN0LndpZHRoO1xyXG4gICAgICAgIHZhciBwb3NYID0gbG5vZGUucmVjdC54O1xyXG4gICAgICAgIHZhciBwb3NZID0gbG5vZGUucmVjdC55O1xyXG4gICAgICAgIHZhciBoID0gbG5vZGUucmVjdC5oZWlnaHQ7XHJcbiAgICAgICAgdmFyIGR1bW15X3BhcmVudF9pZCA9IG51bGw7XHJcbiAgICAgICAgXHJcbiAgICAgICAgLy8gVE9ETzogSXMgaXQgY29ycmVjdD9cclxuICAgICAgICBpZiAoY3lOb2RlLnNjcmF0Y2goJ3NiZ25QZExheW91dCcpICYmIGN5Tm9kZS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnKS5kdW1teV9wYXJlbnRfaWQpXHJcbiAgICAgICAgICAgIGR1bW15X3BhcmVudF9pZCA9IGN5Tm9kZS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnKS5kdW1teV9wYXJlbnRfaWQ7XHJcblxyXG4gICAgICAgIHBEYXRhWyAnbm9kZXMnIF0ucHVzaCh7XHJcbiAgICAgICAgICAgIGlkOiBub2RlSWQsXHJcbiAgICAgICAgICAgIHBpZDogcGFyZW50SWQsXHJcbiAgICAgICAgICAgIHg6IHBvc1gsXHJcbiAgICAgICAgICAgIHk6IHBvc1ksXHJcbiAgICAgICAgICAgIHdpZHRoOiB3LFxyXG4gICAgICAgICAgICBoZWlnaHQ6IGgsXHJcbiAgICAgICAgICAgIGR1bW15X3BhcmVudF9pZDogZHVtbXlfcGFyZW50X2lkXHJcbiAgICAgICAgfSk7XHJcblxyXG4gICAgfVxyXG5cclxuICAgIHZhciBsZWRnZXMgPSBnbS5nZXRBbGxFZGdlcygpO1xyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsZWRnZXMubGVuZ3RoOyBpKyspIHtcclxuICAgICAgICB2YXIgbGVkZ2UgPSBsZWRnZXNbaV07XHJcbiAgICAgICAgdmFyIGVkZ2VJZCA9IGxlZGdlLmlkO1xyXG4gICAgICAgIHZhciBjeUVkZ2UgPSB0aGlzLm9wdGlvbnMuY3kuZ2V0RWxlbWVudEJ5SWQoZWRnZUlkKTtcclxuICAgICAgICB2YXIgc3JjTm9kZUlkID0gY3lFZGdlLnNvdXJjZSgpLmlkKCk7XHJcbiAgICAgICAgdmFyIHRndE5vZGVJZCA9IGN5RWRnZS50YXJnZXQoKS5pZCgpO1xyXG4gICAgICAgIHBEYXRhWyAnZWRnZXMnIF0ucHVzaCh7XHJcbiAgICAgICAgICAgIGlkOiBlZGdlSWQsXHJcbiAgICAgICAgICAgIHNvdXJjZTogc3JjTm9kZUlkLFxyXG4gICAgICAgICAgICB0YXJnZXQ6IHRndE5vZGVJZFxyXG4gICAgICAgIH0pO1xyXG4gICAgfVxyXG5cclxuICAgIHZhciByZWFkeSA9IGZhbHNlO1xyXG5cclxuICAgIHQxLnBhc3MocERhdGEpLnJ1bihmdW5jdGlvbiAocERhdGEpIHtcclxuICAgICAgICB2YXIgbG9nID0gZnVuY3Rpb24gKG1zZykge1xyXG4gICAgICAgICAgICBicm9hZGNhc3Qoe2xvZzogbXNnfSk7XHJcbiAgICAgICAgfTtcclxuXHJcbiAgICAgICAgbG9nKFwic3RhcnQgdGhyZWFkXCIpO1xyXG5cclxuICAgICAgICAvL3RoZSBsYXlvdXQgd2lsbCBiZSBydW4gaW4gdGhlIHRocmVhZCBhbmQgdGhlIHJlc3VsdHMgYXJlIHRvIGJlIHBhc3NlZFxyXG4gICAgICAgIC8vdG8gdGhlIG1haW4gdGhyZWFkIHdpdGggdGhlIHJlc3VsdCBtYXBcclxuICAgICAgICB2YXIgbGF5b3V0X3QgPSBuZXcgU2JnblBETGF5b3V0KCk7XHJcbiAgICAgICAgdmFyIGdtX3QgPSBsYXlvdXRfdC5uZXdHcmFwaE1hbmFnZXIoKTtcclxuICAgICAgICB2YXIgbmdyYXBoID0gZ21fdC5sYXlvdXQubmV3R3JhcGgoKTtcclxuICAgICAgICB2YXIgbm5vZGUgPSBnbV90LmxheW91dC5uZXdOb2RlKG51bGwpO1xyXG4gICAgICAgIHZhciByb290ID0gZ21fdC5hZGQobmdyYXBoLCBubm9kZSk7XHJcbiAgICAgICAgcm9vdC5ncmFwaE1hbmFnZXIgPSBnbV90O1xyXG4gICAgICAgIGdtX3Quc2V0Um9vdEdyYXBoKHJvb3QpO1xyXG4gICAgICAgIHZhciByb290X3QgPSBnbV90LnJvb3RHcmFwaDtcclxuXHJcbiAgICAgICAgLy9tYXBzIGZvciBpbm5lciB1c2FnZSBvZiB0aGUgdGhyZWFkXHJcbiAgICAgICAgdmFyIG9ycGhhbnNfdCA9IFtdO1xyXG4gICAgICAgIHZhciBpZFRvTE5vZGVfdCA9IHt9O1xyXG4gICAgICAgIHZhciBjaGlsZHJlbk1hcCA9IHt9O1xyXG5cclxuICAgICAgICAvL0EgbWFwIG9mIG5vZGUgaWQgdG8gY29ycmVzcG9uZGluZyBub2RlIHBvc2l0aW9uIGFuZCBzaXplc1xyXG4gICAgICAgIC8vaXQgaXMgdG8gYmUgcmV0dXJuZWQgYXQgdGhlIGVuZCBvZiB0aGUgdGhyZWFkIGZ1bmN0aW9uXHJcbiAgICAgICAgdmFyIHJlc3VsdCA9IHt9O1xyXG5cclxuICAgICAgICAvL3RoaXMgZnVuY3Rpb24gaXMgc2ltaWxhciB0byBwcm9jZXNzQ2hpbGRyZW5MaXN0IGZ1bmN0aW9uIGluIHRoZSBtYWluIHRocmVhZFxyXG4gICAgICAgIC8vaXQgaXMgdG8gcHJvY2VzcyB0aGUgbm9kZXMgaW4gY29ycmVjdCBvcmRlciByZWN1cnNpdmVseVxyXG4gICAgICAgIHZhciBwcm9jZXNzTm9kZXMgPSBmdW5jdGlvbiAocGFyZW50LCBjaGlsZHJlbikge1xyXG4gICAgICAgICAgICB2YXIgc2l6ZSA9IGNoaWxkcmVuLmxlbmd0aDtcclxuICAgICAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBzaXplOyBpKyspIHtcclxuICAgICAgICAgICAgICAgIHZhciB0aGVDaGlsZCA9IGNoaWxkcmVuW2ldO1xyXG4gICAgICAgICAgICAgICAgdmFyIGNoaWxkcmVuX29mX2NoaWxkcmVuID0gY2hpbGRyZW5NYXBbdGhlQ2hpbGQuaWRdO1xyXG4gICAgICAgICAgICAgICAgdmFyIHRoZU5vZGU7XHJcblxyXG4gICAgICAgICAgICAgICAgaWYgKHRoZUNoaWxkLndpZHRoICE9IG51bGxcclxuICAgICAgICAgICAgICAgICAgICAgICAgJiYgdGhlQ2hpbGQuaGVpZ2h0ICE9IG51bGwpIHtcclxuICAgICAgICAgICAgICAgICAgICB0aGVOb2RlID0gcGFyZW50LmFkZChuZXcgQ29TRU5vZGUoZ21fdCxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIG5ldyBQb2ludEQodGhlQ2hpbGQueCwgdGhlQ2hpbGQueSksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBuZXcgRGltZW5zaW9uRChwYXJzZUZsb2F0KHRoZUNoaWxkLndpZHRoKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgcGFyc2VGbG9hdCh0aGVDaGlsZC5oZWlnaHQpKSkpO1xyXG4gICAgICAgICAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgICAgICAgICAgICB0aGVOb2RlID0gcGFyZW50LmFkZChuZXcgQ29TRU5vZGUoZ21fdCkpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgdGhlTm9kZS5pZCA9IHRoZUNoaWxkLmlkO1xyXG4gICAgICAgICAgICAgICAgaWRUb0xOb2RlX3RbdGhlQ2hpbGQuaWRdID0gdGhlTm9kZTtcclxuXHJcbiAgICAgICAgICAgICAgICBpZiAoaXNOYU4odGhlTm9kZS5yZWN0LngpKSB7XHJcbiAgICAgICAgICAgICAgICAgICAgdGhlTm9kZS5yZWN0LnggPSAwO1xyXG4gICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgICAgIGlmIChpc05hTih0aGVOb2RlLnJlY3QueSkpIHtcclxuICAgICAgICAgICAgICAgICAgICB0aGVOb2RlLnJlY3QueSA9IDA7XHJcbiAgICAgICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAgICAgaWYgKGNoaWxkcmVuX29mX2NoaWxkcmVuICE9IG51bGwgJiYgY2hpbGRyZW5fb2ZfY2hpbGRyZW4ubGVuZ3RoID4gMCkge1xyXG4gICAgICAgICAgICAgICAgICAgIHZhciB0aGVOZXdHcmFwaDtcclxuICAgICAgICAgICAgICAgICAgICB0aGVOZXdHcmFwaCA9IGxheW91dF90LmdldEdyYXBoTWFuYWdlcigpLmFkZChsYXlvdXRfdC5uZXdHcmFwaCgpLCB0aGVOb2RlKTtcclxuICAgICAgICAgICAgICAgICAgICB0aGVOZXdHcmFwaC5ncmFwaE1hbmFnZXIgPSBnbV90O1xyXG4gICAgICAgICAgICAgICAgICAgIHByb2Nlc3NOb2Rlcyh0aGVOZXdHcmFwaCwgY2hpbGRyZW5fb2ZfY2hpbGRyZW4pO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvL2ZpbGwgdGhlIGNoaWRyZW5NYXAgYW5kIG9ycGhhbnNfdCBtYXBzIHRvIHByb2Nlc3MgdGhlIG5vZGVzIGluIHRoZSBjb3JyZWN0IG9yZGVyXHJcbiAgICAgICAgdmFyIG5vZGVzID0gcERhdGEubm9kZXM7XHJcbiAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKykge1xyXG4gICAgICAgICAgICB2YXIgdGhlTm9kZSA9IG5vZGVzW2ldO1xyXG4gICAgICAgICAgICB2YXIgcF9pZCA9IHRoZU5vZGUucGlkO1xyXG4gICAgICAgICAgICBpZiAocF9pZCAhPSBudWxsKSB7XHJcbiAgICAgICAgICAgICAgICBpZiAoY2hpbGRyZW5NYXBbcF9pZF0gPT0gbnVsbCkge1xyXG4gICAgICAgICAgICAgICAgICAgIGNoaWxkcmVuTWFwW3BfaWRdID0gW107XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICBjaGlsZHJlbk1hcFtwX2lkXS5wdXNoKHRoZU5vZGUpO1xyXG4gICAgICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICAgICAgb3JwaGFuc190LnB1c2godGhlTm9kZSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHByb2Nlc3NOb2Rlcyhyb290X3QsIG9ycGhhbnNfdCk7XHJcblxyXG4gICAgICAgIC8vaGFuZGxlIHRoZSBlZGdlc1xyXG4gICAgICAgIHZhciBlZGdlcyA9IHBEYXRhLmVkZ2VzO1xyXG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZWRnZXMubGVuZ3RoOyBpKyspIHtcclxuICAgICAgICAgICAgdmFyIGVkZ2UgPSBlZGdlc1tpXTtcclxuICAgICAgICAgICAgdmFyIHNvdXJjZU5vZGUgPSBpZFRvTE5vZGVfdFtlZGdlLnNvdXJjZV07XHJcbiAgICAgICAgICAgIHZhciB0YXJnZXROb2RlID0gaWRUb0xOb2RlX3RbZWRnZS50YXJnZXRdO1xyXG4gICAgICAgICAgICB2YXIgZTEgPSBnbV90LmFkZChsYXlvdXRfdC5uZXdFZGdlKCksIHNvdXJjZU5vZGUsIHRhcmdldE5vZGUpO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy9ydW4gdGhlIGxheW91dCBjcmF0ZWQgaW4gdGhpcyB0aHJlYWRcclxuICAgICAgICBsYXlvdXRfdC5ydW5MYXlvdXQoKTtcclxuXHJcbiAgICAgICAgLy9maWxsIHRoZSByZXN1bHQgbWFwXHJcbiAgICAgICAgZm9yICh2YXIgaWQgaW4gaWRUb0xOb2RlX3QpIHtcclxuICAgICAgICAgICAgdmFyIGxOb2RlID0gaWRUb0xOb2RlX3RbaWRdO1xyXG4gICAgICAgICAgICB2YXIgcmVjdCA9IGxOb2RlLnJlY3Q7XHJcbiAgICAgICAgICAgIHJlc3VsdFtpZF0gPSB7XHJcbiAgICAgICAgICAgICAgICBpZDogaWQsXHJcbiAgICAgICAgICAgICAgICB4OiByZWN0LngsXHJcbiAgICAgICAgICAgICAgICB5OiByZWN0LnksXHJcbiAgICAgICAgICAgICAgICB3OiByZWN0LndpZHRoLFxyXG4gICAgICAgICAgICAgICAgaDogcmVjdC5oZWlnaHRcclxuICAgICAgICAgICAgfTtcclxuICAgICAgICB9XHJcbiAgICAgICAgdmFyIHNlZWRzID0ge307XHJcbiAgICAgICAgc2VlZHMucnNTZWVkID0gUmFuZG9tU2VlZC5zZWVkO1xyXG4gICAgICAgIHNlZWRzLnJzWCA9IFJhbmRvbVNlZWQueDtcclxuICAgICAgICB2YXIgcGFzcyA9IHtcclxuICAgICAgICAgICAgcmVzdWx0OiByZXN1bHQsXHJcbiAgICAgICAgICAgIHNlZWRzOiBzZWVkc1xyXG4gICAgICAgIH1cclxuICAgICAgICAvL3JldHVybiB0aGUgcmVzdWx0IG1hcCB0byBwYXNzIGl0IHRvIHRoZSB0aGVuIGZ1bmN0aW9uIGFzIHBhcmFtZXRlclxyXG4gICAgICAgIHJldHVybiBwYXNzO1xyXG4gICAgfSkudGhlbihmdW5jdGlvbiAocGFzcykge1xyXG4gICAgICAgIHZhciByZXN1bHQgPSBwYXNzLnJlc3VsdDtcclxuICAgICAgICB2YXIgc2VlZHMgPSBwYXNzLnNlZWRzO1xyXG4gICAgICAgIFJhbmRvbVNlZWQuc2VlZCA9IHNlZWRzLnJzU2VlZDtcclxuICAgICAgICBSYW5kb21TZWVkLnggPSBzZWVkcy5yc1g7XHJcbiAgICAgICAgLy9yZWZyZXNoIHRoZSBsbm9kZSBwb3NpdGlvbnMgYW5kIHNpemVzIGJ5IHVzaW5nIHJlc3VsdCBtYXBcclxuICAgICAgICBmb3IgKHZhciBpZCBpbiByZXN1bHQpIHtcclxuICAgICAgICAgICAgdmFyIGxOb2RlID0gX1NiZ25QRExheW91dC5pZFRvTE5vZGVbaWRdO1xyXG4gICAgICAgICAgICB2YXIgbm9kZSA9IHJlc3VsdFtpZF07XHJcbiAgICAgICAgICAgIGxOb2RlLnJlY3QueCA9IG5vZGUueDtcclxuICAgICAgICAgICAgbE5vZGUucmVjdC55ID0gbm9kZS55O1xyXG4gICAgICAgICAgICBsTm9kZS5yZWN0LndpZHRoID0gbm9kZS53O1xyXG4gICAgICAgICAgICBsTm9kZS5yZWN0LmhlaWdodCA9IG5vZGUuaDtcclxuICAgICAgICB9XHJcbiAgICAgICAgaWYgKGFmdGVyLm9wdGlvbnMudGlsZSkge1xyXG4gICAgICAgICAgICAvLyBSZXBvcHVsYXRlIG1lbWJlcnNcclxuICAgICAgICAgICAgYWZ0ZXIucmVwb3B1bGF0ZVplcm9EZWdyZWVNZW1iZXJzKHRpbGVkWmVyb0RlZ3JlZU5vZGVzKTtcclxuICAgICAgICAgICAgYWZ0ZXIucmVwb3B1bGF0ZUNvbXBvdW5kcyh0aWxlZE1lbWJlclBhY2spO1xyXG4gICAgICAgICAgICBhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKS51cGRhdGVDb21wb3VuZEJvdW5kcygpO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgdmFyIGdldFBvc2l0aW9ucyA9IGZ1bmN0aW9uIChpLCBlbGUpIHtcclxuICAgICAgICAgICAgdmFyIHRoZUlkID0gZWxlLmRhdGEoJ2lkJyk7XHJcbiAgICAgICAgICAgIHZhciBsTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW3RoZUlkXTtcclxuXHJcbiAgICAgICAgICAgIHJldHVybiB7XHJcbiAgICAgICAgICAgICAgICB4OiBsTm9kZS5nZXRSZWN0KCkuZ2V0Q2VudGVyWCgpLFxyXG4gICAgICAgICAgICAgICAgeTogbE5vZGUuZ2V0UmVjdCgpLmdldENlbnRlclkoKVxyXG4gICAgICAgICAgICB9O1xyXG4gICAgICAgIH07XHJcblxyXG4gICAgICAgIGlmIChhZnRlci5vcHRpb25zLmFuaW1hdGUgIT09ICdkdXJpbmcnKSB7XHJcbiAgICAgICAgICAgIGFmdGVyLm9wdGlvbnMuZWxlcy5ub2RlcygpLmxheW91dFBvc2l0aW9ucyhhZnRlciwgYWZ0ZXIub3B0aW9ucywgZ2V0UG9zaXRpb25zKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICBhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKS5wb3NpdGlvbnMoZ2V0UG9zaXRpb25zKTtcclxuXHJcbiAgICAgICAgICAgIGlmIChhZnRlci5vcHRpb25zLmZpdClcclxuICAgICAgICAgICAgICAgIGFmdGVyLm9wdGlvbnMuY3kuZml0KGFmdGVyLm9wdGlvbnMuZWxlcy5ub2RlcygpLCBhZnRlci5vcHRpb25zLnBhZGRpbmcpO1xyXG5cclxuICAgICAgICAgICAgLy90cmlnZ2VyIGxheW91dHJlYWR5IHdoZW4gZWFjaCBub2RlIGhhcyBoYWQgaXRzIHBvc2l0aW9uIHNldCBhdCBsZWFzdCBvbmNlXHJcbiAgICAgICAgICAgIGlmICghcmVhZHkpIHtcclxuICAgICAgICAgICAgICAgIGFmdGVyLmN5Lm9uZSgnbGF5b3V0cmVhZHknLCBhZnRlci5vcHRpb25zLnJlYWR5KTtcclxuICAgICAgICAgICAgICAgIGFmdGVyLmN5LnRyaWdnZXIoJ2xheW91dHJlYWR5Jyk7XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIC8vIHRyaWdnZXIgbGF5b3V0c3RvcCB3aGVuIHRoZSBsYXlvdXQgc3RvcHMgKGUuZy4gZmluaXNoZXMpXHJcbiAgICAgICAgICAgIGFmdGVyLmN5Lm9uZSgnbGF5b3V0c3RvcCcsIGFmdGVyLm9wdGlvbnMuc3RvcCk7XHJcbiAgICAgICAgICAgIGFmdGVyLmN5LnRyaWdnZXIoJ2xheW91dHN0b3AnKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHQxLnN0b3AoKTtcclxuICAgICAgICBhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKS5yZW1vdmVTY3JhdGNoKCdzYmduUGRMYXlvdXQnKTtcclxuICAgIH0pO1xyXG5cclxuICAgIHQxLm9uKCdtZXNzYWdlJywgZnVuY3Rpb24gKGUpIHtcclxuICAgICAgICB2YXIgbG9nTXNnID0gZS5tZXNzYWdlLmxvZztcclxuICAgICAgICBpZiAobG9nTXNnICE9IG51bGwpIHtcclxuICAgICAgICAgICAgY29uc29sZS5sb2coJ1RocmVhZCBsb2c6ICcgKyBsb2dNc2cpO1xyXG4gICAgICAgICAgICByZXR1cm47XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHZhciBwRGF0YSA9IGUubWVzc2FnZS5wRGF0YTtcclxuICAgICAgICBpZiAocERhdGEgIT0gbnVsbCkge1xyXG4gICAgICAgICAgICBhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKS5wb3NpdGlvbnMoZnVuY3Rpb24gKGksIGVsZSkge1xyXG4gICAgICAgICAgICAgICAgaWYgKGVsZS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnKSAmJiBlbGUuc2NyYXRjaCgnc2JnblBkTGF5b3V0JykuZHVtbXlfcGFyZW50X2lkKSB7XHJcbiAgICAgICAgICAgICAgICAgICAgdmFyIGR1bW15UGFyZW50ID0gZWxlLnNjcmF0Y2goJ3NiZ25QZExheW91dCcpLmR1bW15X3BhcmVudF9pZDtcclxuICAgICAgICAgICAgICAgICAgICByZXR1cm4ge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICB4OiBkdW1teVBhcmVudC54LFxyXG4gICAgICAgICAgICAgICAgICAgICAgICB5OiBkdW1teVBhcmVudC55XHJcbiAgICAgICAgICAgICAgICAgICAgfTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIHZhciB0aGVJZCA9IGVsZS5kYXRhKCdpZCcpO1xyXG4gICAgICAgICAgICAgICAgdmFyIHBOb2RlID0gcERhdGFbdGhlSWRdO1xyXG4gICAgICAgICAgICAgICAgdmFyIHRlbXAgPSB0aGlzO1xyXG4gICAgICAgICAgICAgICAgd2hpbGUgKHBOb2RlID09IG51bGwpIHtcclxuICAgICAgICAgICAgICAgICAgICB0ZW1wID0gdGVtcC5wYXJlbnQoKVswXTtcclxuICAgICAgICAgICAgICAgICAgICBwTm9kZSA9IHBEYXRhW3RlbXAuaWQoKV07XHJcbiAgICAgICAgICAgICAgICAgICAgcERhdGFbdGhlSWRdID0gcE5vZGU7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICByZXR1cm4ge1xyXG4gICAgICAgICAgICAgICAgICAgIHg6IHBOb2RlLngsXHJcbiAgICAgICAgICAgICAgICAgICAgeTogcE5vZGUueVxyXG4gICAgICAgICAgICAgICAgfTtcclxuICAgICAgICAgICAgfSk7XHJcblxyXG4gICAgICAgICAgICBpZiAoYWZ0ZXIub3B0aW9ucy5maXQpXHJcbiAgICAgICAgICAgICAgICBhZnRlci5vcHRpb25zLmN5LmZpdChhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKSwgYWZ0ZXIub3B0aW9ucy5wYWRkaW5nKTtcclxuXHJcbiAgICAgICAgICAgIGlmICghcmVhZHkpIHtcclxuICAgICAgICAgICAgICAgIHJlYWR5ID0gdHJ1ZTtcclxuICAgICAgICAgICAgICAgIGFmdGVyLm9uZSgnbGF5b3V0cmVhZHknLCBhZnRlci5vcHRpb25zLnJlYWR5KTtcclxuICAgICAgICAgICAgICAgIGFmdGVyLnRyaWdnZXIoe3R5cGU6ICdsYXlvdXRyZWFkeScsIGxheW91dDogYWZ0ZXJ9KTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICByZXR1cm47XHJcbiAgICAgICAgfVxyXG4gICAgfSk7XHJcblxyXG4gICAgcmV0dXJuIHRoaXM7IC8vIGNoYWluaW5nXHJcbn07XHJcblxyXG4vL0dldCB0aGUgdG9wIG1vc3Qgb25lcyBvZiBhIGxpc3Qgb2Ygbm9kZXNcclxuX1NiZ25QRExheW91dC5nZXRUb3BNb3N0Tm9kZXMgPSBmdW5jdGlvbiAobm9kZXMpIHtcclxuICAgIHZhciBub2Rlc01hcCA9IHt9O1xyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKykge1xyXG4gICAgICAgIG5vZGVzTWFwW25vZGVzW2ldLmlkKCldID0gdHJ1ZTtcclxuICAgIH1cclxuICAgIHZhciByb290cyA9IG5vZGVzLmZpbHRlcihmdW5jdGlvbiAoaSwgZWxlKSB7XHJcbiAgICAgICAgdmFyIHBhcmVudCA9IGVsZS5wYXJlbnQoKVswXTtcclxuICAgICAgICB3aGlsZSAocGFyZW50ICE9IG51bGwpIHtcclxuICAgICAgICAgICAgaWYgKG5vZGVzTWFwW3BhcmVudC5pZCgpXSkge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIHBhcmVudCA9IHBhcmVudC5wYXJlbnQoKVswXTtcclxuICAgICAgICB9XHJcbiAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICB9KTtcclxuXHJcbiAgICByZXR1cm4gcm9vdHM7XHJcbn07XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmdldFRvQmVUaWxlZCA9IGZ1bmN0aW9uIChub2RlKSB7XHJcbi8vICAgIHZhciBpZCA9IG5vZGUuZGF0YShcImlkXCIpO1xyXG4vLyAgICAvL2ZpcnN0bHkgY2hlY2sgdGhlIHByZXZpb3VzIHJlc3VsdHNcclxuLy8gICAgaWYgKF9TYmduUERMYXlvdXQudG9CZVRpbGVkW2lkXSAhPSBudWxsKSB7XHJcbi8vICAgICAgICByZXR1cm4gX1NiZ25QRExheW91dC50b0JlVGlsZWRbaWRdO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIC8vb25seSBjb21wb3VuZCBub2RlcyBhcmUgdG8gYmUgdGlsZWRcclxuLy8gICAgdmFyIGNoaWxkcmVuID0gbm9kZS5jaGlsZHJlbigpO1xyXG4vLyAgICBpZiAoY2hpbGRyZW4gPT0gbnVsbCB8fCBjaGlsZHJlbi5sZW5ndGggPT0gMCkge1xyXG4vLyAgICAgICAgX1NiZ25QRExheW91dC50b0JlVGlsZWRbaWRdID0gZmFsc2U7XHJcbi8vICAgICAgICByZXR1cm4gZmFsc2U7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy9hIGNvbXBvdW5kIG5vZGUgaXMgbm90IHRvIGJlIHRpbGVkIGlmIGFsbCBvZiBpdHMgY29tcG91bmQgY2hpbGRyZW4gYXJlIG5vdCB0byBiZSB0aWxlZFxyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IGNoaWxkcmVuLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICB2YXIgdGhlQ2hpbGQgPSBjaGlsZHJlbltpXTtcclxuLy9cclxuLy8gICAgICAgIGlmICh0aGlzLmdldE5vZGVEZWdyZWUodGhlQ2hpbGQpID4gMCkge1xyXG4vLyAgICAgICAgICAgIF9TYmduUERMYXlvdXQudG9CZVRpbGVkW2lkXSA9IGZhbHNlO1xyXG4vLyAgICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuLy8gICAgICAgIH1cclxuLy9cclxuLy8gICAgICAgIC8vcGFzcyB0aGUgY2hpbGRyZW4gbm90IGhhdmluZyB0aGUgY29tcG91bmQgc3RydWN0dXJlXHJcbi8vICAgICAgICBpZiAodGhlQ2hpbGQuY2hpbGRyZW4oKSA9PSBudWxsIHx8IHRoZUNoaWxkLmNoaWxkcmVuKCkubGVuZ3RoID09IDApIHtcclxuLy8gICAgICAgICAgICBfU2JnblBETGF5b3V0LnRvQmVUaWxlZFt0aGVDaGlsZC5kYXRhKFwiaWRcIildID0gZmFsc2U7XHJcbi8vICAgICAgICAgICAgY29udGludWU7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICBpZiAoIXRoaXMuZ2V0VG9CZVRpbGVkKHRoZUNoaWxkKSkge1xyXG4vLyAgICAgICAgICAgIF9TYmduUERMYXlvdXQudG9CZVRpbGVkW2lkXSA9IGZhbHNlO1xyXG4vLyAgICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuLy8gICAgICAgIH1cclxuLy8gICAgfVxyXG4vLyAgICBfU2JnblBETGF5b3V0LnRvQmVUaWxlZFtpZF0gPSB0cnVlO1xyXG4vLyAgICByZXR1cm4gdHJ1ZTtcclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5nZXROb2RlRGVncmVlID0gZnVuY3Rpb24gKG5vZGUpIHtcclxuLy8gICAgdmFyIGlkID0gbm9kZS5pZCgpO1xyXG4vLyAgICB2YXIgZWRnZXMgPSB0aGlzLm9wdGlvbnMuZWxlcy5lZGdlcygpLmZpbHRlcihmdW5jdGlvbiAoaSwgZWxlKSB7XHJcbi8vICAgICAgICB2YXIgc291cmNlID0gZWxlLmRhdGEoJ3NvdXJjZScpO1xyXG4vLyAgICAgICAgdmFyIHRhcmdldCA9IGVsZS5kYXRhKCd0YXJnZXQnKTtcclxuLy8gICAgICAgIGlmIChzb3VyY2UgIT0gdGFyZ2V0ICYmIChzb3VyY2UgPT0gaWQgfHwgdGFyZ2V0ID09IGlkKSkge1xyXG4vLyAgICAgICAgICAgIHJldHVybiB0cnVlO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9KTtcclxuLy8gICAgcmV0dXJuIGVkZ2VzLmxlbmd0aDtcclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5nZXROb2RlRGVncmVlV2l0aENoaWxkcmVuID0gZnVuY3Rpb24gKG5vZGUpIHtcclxuLy8gICAgdmFyIGRlZ3JlZSA9IHRoaXMuZ2V0Tm9kZURlZ3JlZShub2RlKTtcclxuLy8gICAgdmFyIGNoaWxkcmVuID0gbm9kZS5jaGlsZHJlbigpO1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IGNoaWxkcmVuLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICB2YXIgY2hpbGQgPSBjaGlsZHJlbltpXTtcclxuLy8gICAgICAgIGRlZ3JlZSArPSB0aGlzLmdldE5vZGVEZWdyZWVXaXRoQ2hpbGRyZW4oY2hpbGQpO1xyXG4vLyAgICB9XHJcbi8vICAgIHJldHVybiBkZWdyZWU7XHJcbi8vfTtcclxuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuZ3JvdXBaZXJvRGVncmVlTWVtYmVycyA9IGZ1bmN0aW9uICgpIHtcclxuLy8gICAgLy8gYXJyYXkgb2YgW3BhcmVudF9pZCB4IG9uZURlZ3JlZU5vZGVfaWRdIFxyXG4vLyAgICB2YXIgdGVtcE1lbWJlckdyb3VwcyA9IFtdO1xyXG4vLyAgICB2YXIgbWVtYmVyR3JvdXBzID0gW107XHJcbi8vICAgIHZhciBzZWxmID0gdGhpcztcclxuLy8gICAgdmFyIHBhcmVudE1hcCA9IHt9O1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMub3B0aW9ucy5lbGVzLm5vZGVzKCkubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgIHBhcmVudE1hcFt0aGlzLm9wdGlvbnMuZWxlcy5ub2RlcygpW2ldLmlkKCldID0gdHJ1ZTtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICAvLyBGaW5kIGFsbCB6ZXJvIGRlZ3JlZSBub2RlcyB3aGljaCBhcmVuJ3QgY292ZXJlZCBieSBhIGNvbXBvdW5kXHJcbi8vICAgIHZhciB6ZXJvRGVncmVlID0gdGhpcy5vcHRpb25zLmVsZXMubm9kZXMoKS5maWx0ZXIoZnVuY3Rpb24gKGksIGVsZSkge1xyXG4vLyAgICAgICAgdmFyIHBpZCA9IGVsZS5kYXRhKCdwYXJlbnQnKTtcclxuLy8gICAgICAgIGlmIChwaWQgIT0gdW5kZWZpbmVkICYmICFwYXJlbnRNYXBbcGlkXSkge1xyXG4vLyAgICAgICAgICAgIHBpZCA9IHVuZGVmaW5lZDtcclxuLy8gICAgICAgIH1cclxuLy9cclxuLy8gICAgICAgIGlmIChzZWxmLmdldE5vZGVEZWdyZWVXaXRoQ2hpbGRyZW4oZWxlKSA9PSAwICYmIChwaWQgPT0gdW5kZWZpbmVkIHx8IChwaWQgIT0gdW5kZWZpbmVkICYmICFzZWxmLmdldFRvQmVUaWxlZChlbGUucGFyZW50KClbMF0pKSkpXHJcbi8vICAgICAgICAgICAgcmV0dXJuIHRydWU7XHJcbi8vICAgICAgICBlbHNlXHJcbi8vICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4vLyAgICB9KTtcclxuLy9cclxuLy8gICAgLy8gQ3JlYXRlIGEgbWFwIG9mIHBhcmVudCBub2RlIGFuZCBpdHMgemVybyBkZWdyZWUgbWVtYmVyc1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IHplcm9EZWdyZWUubGVuZ3RoOyBpKyspXHJcbi8vICAgIHtcclxuLy8gICAgICAgIHZhciBub2RlID0gemVyb0RlZ3JlZVtpXTtcclxuLy8gICAgICAgIHZhciBwX2lkID0gbm9kZS5wYXJlbnQoKS5pZCgpO1xyXG4vL1xyXG4vLyAgICAgICAgaWYgKHBfaWQgIT0gdW5kZWZpbmVkICYmICFwYXJlbnRNYXBbcF9pZF0pIHtcclxuLy8gICAgICAgICAgICBwX2lkID0gdW5kZWZpbmVkO1xyXG4vLyAgICAgICAgfVxyXG4vL1xyXG4vLyAgICAgICAgaWYgKHR5cGVvZiB0ZW1wTWVtYmVyR3JvdXBzW3BfaWRdID09PSBcInVuZGVmaW5lZFwiKVxyXG4vLyAgICAgICAgICAgIHRlbXBNZW1iZXJHcm91cHNbcF9pZF0gPSBbXTtcclxuLy9cclxuLy8gICAgICAgIHRlbXBNZW1iZXJHcm91cHNbcF9pZF0gPSB0ZW1wTWVtYmVyR3JvdXBzW3BfaWRdLmNvbmNhdChub2RlKTtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICAvLyBJZiB0aGVyZSBhcmUgYXQgbGVhc3QgdHdvIG5vZGVzIGF0IGEgbGV2ZWwsIGNyZWF0ZSBhIGR1bW15IGNvbXBvdW5kIGZvciB0aGVtXHJcbi8vICAgIGZvciAodmFyIHBfaWQgaW4gdGVtcE1lbWJlckdyb3Vwcykge1xyXG4vLyAgICAgICAgaWYgKHRlbXBNZW1iZXJHcm91cHNbcF9pZF0ubGVuZ3RoID4gMSkge1xyXG4vLyAgICAgICAgICAgIHZhciBkdW1teUNvbXBvdW5kSWQgPSBcIkR1bW15Q29tcG91bmRfXCIgKyBwX2lkO1xyXG4vLyAgICAgICAgICAgIG1lbWJlckdyb3Vwc1tkdW1teUNvbXBvdW5kSWRdID0gdGVtcE1lbWJlckdyb3Vwc1twX2lkXTtcclxuLy9cclxuLy8gICAgICAgICAgICAvLyBDcmVhdGUgYSBkdW1teSBjb21wb3VuZFxyXG4vLyAgICAgICAgICAgIGlmICh0aGlzLm9wdGlvbnMuY3kuZ2V0RWxlbWVudEJ5SWQoZHVtbXlDb21wb3VuZElkKS5lbXB0eSgpKSB7XHJcbi8vICAgICAgICAgICAgICAgIHRoaXMub3B0aW9ucy5jeS5hZGQoe1xyXG4vLyAgICAgICAgICAgICAgICAgICAgZ3JvdXA6IFwibm9kZXNcIixcclxuLy8gICAgICAgICAgICAgICAgICAgIGRhdGE6IHtpZDogZHVtbXlDb21wb3VuZElkLCBwYXJlbnQ6IHBfaWRcclxuLy8gICAgICAgICAgICAgICAgICAgIH1cclxuLy8gICAgICAgICAgICAgICAgfSk7XHJcbi8vXHJcbi8vICAgICAgICAgICAgICAgIHZhciBkdW1teSA9IHRoaXMub3B0aW9ucy5jeS5ub2RlcygpW3RoaXMub3B0aW9ucy5jeS5ub2RlcygpLmxlbmd0aCAtIDFdO1xyXG4vLyAgICAgICAgICAgICAgICB0aGlzLm9wdGlvbnMuZWxlcyA9IHRoaXMub3B0aW9ucy5lbGVzLnVuaW9uKGR1bW15KTtcclxuLy8gICAgICAgICAgICAgICAgZHVtbXkuaGlkZSgpO1xyXG4vL1xyXG4vLyAgICAgICAgICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRlbXBNZW1iZXJHcm91cHNbcF9pZF0ubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgICAgICAgICAgICAgIGlmIChpID09IDApIHtcclxuLy8gICAgICAgICAgICAgICAgICAgICAgICBkdW1teS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnLCB7dGVtcGNoaWxkcmVuOiBbXX0pO1xyXG4vLyAgICAgICAgICAgICAgICAgICAgfVxyXG4vLyAgICAgICAgICAgICAgICAgICAgdmFyIG5vZGUgPSB0ZW1wTWVtYmVyR3JvdXBzW3BfaWRdW2ldO1xyXG4vLyAgICAgICAgICAgICAgICAgICAgdmFyIHNjcmF0Y2hPYmogPSBub2RlLnNjcmF0Y2goJ3NiZ25QZExheW91dCcpO1xyXG4vLyAgICAgICAgICAgICAgICAgICAgaWYgKCFzY3JhdGNoT2JqKSB7XHJcbi8vICAgICAgICAgICAgICAgICAgICAgICAgc2NyYXRjaE9iaiA9IHt9O1xyXG4vLyAgICAgICAgICAgICAgICAgICAgICAgIG5vZGUuc2NyYXRjaCgnc2JnblBkTGF5b3V0Jywgc2NyYXRjaE9iaik7XHJcbi8vICAgICAgICAgICAgICAgICAgICB9XHJcbi8vICAgICAgICAgICAgICAgICAgICBzY3JhdGNoT2JqWydkdW1teV9wYXJlbnRfaWQnXSA9IGR1bW15Q29tcG91bmRJZDtcclxuLy8gICAgICAgICAgICAgICAgICAgIHRoaXMub3B0aW9ucy5jeS5hZGQoe1xyXG4vLyAgICAgICAgICAgICAgICAgICAgICAgIGdyb3VwOiBcIm5vZGVzXCIsXHJcbi8vICAgICAgICAgICAgICAgICAgICAgICAgZGF0YToge3BhcmVudDogZHVtbXlDb21wb3VuZElkLCB3aWR0aDogbm9kZS53aWR0aCgpLCBoZWlnaHQ6IG5vZGUuaGVpZ2h0KClcclxuLy8gICAgICAgICAgICAgICAgICAgICAgICB9XHJcbi8vICAgICAgICAgICAgICAgICAgICB9KTtcclxuLy8gICAgICAgICAgICAgICAgICAgIHZhciB0ZW1wY2hpbGQgPSB0aGlzLm9wdGlvbnMuY3kubm9kZXMoKVt0aGlzLm9wdGlvbnMuY3kubm9kZXMoKS5sZW5ndGggLSAxXTtcclxuLy8gICAgICAgICAgICAgICAgICAgIHRlbXBjaGlsZC5oaWRlKCk7XHJcbi8vICAgICAgICAgICAgICAgICAgICB0ZW1wY2hpbGQuY3NzKCd3aWR0aCcsIHRlbXBjaGlsZC5kYXRhKCd3aWR0aCcpKTtcclxuLy8gICAgICAgICAgICAgICAgICAgIHRlbXBjaGlsZC5jc3MoJ2hlaWdodCcsIHRlbXBjaGlsZC5kYXRhKCdoZWlnaHQnKSk7XHJcbi8vICAgICAgICAgICAgICAgICAgICB0ZW1wY2hpbGQud2lkdGgoKTtcclxuLy8gICAgICAgICAgICAgICAgICAgIGR1bW15LnNjcmF0Y2goJ3NiZ25QZExheW91dCcpLnRlbXBjaGlsZHJlbi5wdXNoKHRlbXBjaGlsZCk7XHJcbi8vICAgICAgICAgICAgICAgIH1cclxuLy8gICAgICAgICAgICB9XHJcbi8vICAgICAgICB9XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgcmV0dXJuIG1lbWJlckdyb3VwcztcclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5wZXJmb3JtREZTT25Db21wb3VuZHMgPSBmdW5jdGlvbiAob3B0aW9ucykge1xyXG4vLyAgICB2YXIgY29tcG91bmRPcmRlciA9IFtdO1xyXG4vL1xyXG4vLyAgICB2YXIgcm9vdHMgPSBfU2JnblBETGF5b3V0LmdldFRvcE1vc3ROb2Rlcyh0aGlzLm9wdGlvbnMuZWxlcy5ub2RlcygpKTtcclxuLy8gICAgdGhpcy5maWxsQ29tcGV4T3JkZXJCeURGUyhjb21wb3VuZE9yZGVyLCByb290cyk7XHJcbi8vXHJcbi8vICAgIHJldHVybiBjb21wb3VuZE9yZGVyO1xyXG4vL307XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmZpbGxDb21wZXhPcmRlckJ5REZTID0gZnVuY3Rpb24gKGNvbXBvdW5kT3JkZXIsIGNoaWxkcmVuKSB7XHJcbi8vICAgIGZvciAodmFyIGkgPSAwOyBpIDwgY2hpbGRyZW4ubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgIHZhciBjaGlsZCA9IGNoaWxkcmVuW2ldO1xyXG4vLyAgICAgICAgdGhpcy5maWxsQ29tcGV4T3JkZXJCeURGUyhjb21wb3VuZE9yZGVyLCBjaGlsZC5jaGlsZHJlbigpKTtcclxuLy8gICAgICAgIGlmICh0aGlzLmdldFRvQmVUaWxlZChjaGlsZCkpIHtcclxuLy8gICAgICAgICAgICBjb21wb3VuZE9yZGVyLnB1c2goY2hpbGQpO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9XHJcbi8vfTtcclxuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuY2xlYXJDb21wb3VuZHMgPSBmdW5jdGlvbiAob3B0aW9ucykge1xyXG4vLyAgICB2YXIgY2hpbGRHcmFwaE1hcCA9IFtdO1xyXG4vL1xyXG4vLyAgICAvLyBHZXQgY29tcG91bmQgb3JkZXJpbmcgYnkgZmluZGluZyB0aGUgaW5uZXIgb25lIGZpcnN0XHJcbi8vICAgIHZhciBjb21wb3VuZE9yZGVyID0gdGhpcy5wZXJmb3JtREZTT25Db21wb3VuZHMob3B0aW9ucyk7XHJcbi8vICAgIF9TYmduUERMYXlvdXQuY29tcG91bmRPcmRlciA9IGNvbXBvdW5kT3JkZXI7XHJcbi8vICAgIHRoaXMucHJvY2Vzc0NoaWxkcmVuTGlzdCh0aGlzLnJvb3QsIF9TYmduUERMYXlvdXQuZ2V0VG9wTW9zdE5vZGVzKHRoaXMub3B0aW9ucy5lbGVzLm5vZGVzKCkpKTtcclxuLy9cclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBjb21wb3VuZE9yZGVyLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICAvLyBmaW5kIHRoZSBjb3JyZXNwb25kaW5nIGxheW91dCBub2RlXHJcbi8vICAgICAgICB2YXIgbENvbXBvdW5kTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2NvbXBvdW5kT3JkZXJbaV0uaWQoKV07XHJcbi8vXHJcbi8vICAgICAgICBjaGlsZEdyYXBoTWFwW2NvbXBvdW5kT3JkZXJbaV0uaWQoKV0gPSBjb21wb3VuZE9yZGVyW2ldLmNoaWxkcmVuKCk7XHJcbi8vXHJcbi8vICAgICAgICAvLyBSZW1vdmUgY2hpbGRyZW4gb2YgY29tcG91bmRzIFxyXG4vLyAgICAgICAgbENvbXBvdW5kTm9kZS5jaGlsZCA9IG51bGw7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy8gVGlsZSB0aGUgcmVtb3ZlZCBjaGlsZHJlblxyXG4vLyAgICB2YXIgdGlsZWRNZW1iZXJQYWNrID0gdGhpcy50aWxlQ29tcG91bmRNZW1iZXJzKGNoaWxkR3JhcGhNYXApO1xyXG4vL1xyXG4vLyAgICByZXR1cm4gdGlsZWRNZW1iZXJQYWNrO1xyXG4vL307XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmNsZWFyWmVyb0RlZ3JlZU1lbWJlcnMgPSBmdW5jdGlvbiAobWVtYmVyR3JvdXBzKSB7XHJcbi8vICAgIHZhciB0aWxlZFplcm9EZWdyZWVQYWNrID0gW107XHJcbi8vXHJcbi8vICAgIGZvciAodmFyIGlkIGluIG1lbWJlckdyb3Vwcykge1xyXG4vLyAgICAgICAgdmFyIGNvbXBvdW5kTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2lkXTtcclxuLy9cclxuLy8gICAgICAgIHRpbGVkWmVyb0RlZ3JlZVBhY2tbaWRdID0gdGhpcy50aWxlTm9kZXMobWVtYmVyR3JvdXBzW2lkXSk7XHJcbi8vXHJcbi8vICAgICAgICAvLyBTZXQgdGhlIHdpZHRoIGFuZCBoZWlnaHQgb2YgdGhlIGR1bW15IGNvbXBvdW5kIGFzIGNhbGN1bGF0ZWRcclxuLy8gICAgICAgIGNvbXBvdW5kTm9kZS5yZWN0LndpZHRoID0gdGlsZWRaZXJvRGVncmVlUGFja1tpZF0ud2lkdGg7XHJcbi8vICAgICAgICBjb21wb3VuZE5vZGUucmVjdC5oZWlnaHQgPSB0aWxlZFplcm9EZWdyZWVQYWNrW2lkXS5oZWlnaHQ7XHJcbi8vICAgIH1cclxuLy8gICAgcmV0dXJuIHRpbGVkWmVyb0RlZ3JlZVBhY2s7XHJcbi8vfTtcclxuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUucmVwb3B1bGF0ZUNvbXBvdW5kcyA9IGZ1bmN0aW9uICh0aWxlZE1lbWJlclBhY2spIHtcclxuLy8gICAgZm9yICh2YXIgaSA9IF9TYmduUERMYXlvdXQuY29tcG91bmRPcmRlci5sZW5ndGggLSAxOyBpID49IDA7IGktLSkge1xyXG4vLyAgICAgICAgdmFyIGlkID0gX1NiZ25QRExheW91dC5jb21wb3VuZE9yZGVyW2ldLmlkKCk7XHJcbi8vICAgICAgICB2YXIgbENvbXBvdW5kTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2lkXTtcclxuLy8gICAgICAgIHZhciBob3Jpem9udGFsTWFyZ2luID0gcGFyc2VJbnQoX1NiZ25QRExheW91dC5jb21wb3VuZE9yZGVyW2ldLmNzcygncGFkZGluZy1sZWZ0JykpO1xyXG4vLyAgICAgICAgdmFyIHZlcnRpY2FsTWFyZ2luID0gcGFyc2VJbnQoX1NiZ25QRExheW91dC5jb21wb3VuZE9yZGVyW2ldLmNzcygncGFkZGluZy10b3AnKSk7XHJcbi8vXHJcbi8vICAgICAgICB0aGlzLmFkanVzdExvY2F0aW9ucyh0aWxlZE1lbWJlclBhY2tbaWRdLCBsQ29tcG91bmROb2RlLnJlY3QueCwgbENvbXBvdW5kTm9kZS5yZWN0LnksIGhvcml6b250YWxNYXJnaW4sIHZlcnRpY2FsTWFyZ2luKTtcclxuLy8gICAgfVxyXG4vL307XHJcbi8vXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUucmVwb3B1bGF0ZVplcm9EZWdyZWVNZW1iZXJzID0gZnVuY3Rpb24gKHRpbGVkUGFjaykge1xyXG4vLyAgICBmb3IgKHZhciBpIGluIHRpbGVkUGFjaykge1xyXG4vLyAgICAgICAgdmFyIGNvbXBvdW5kID0gdGhpcy5jeS5nZXRFbGVtZW50QnlJZChpKTtcclxuLy8gICAgICAgIHZhciBjb21wb3VuZE5vZGUgPSBfU2JnblBETGF5b3V0LmlkVG9MTm9kZVtpXTtcclxuLy8gICAgICAgIHZhciBob3Jpem9udGFsTWFyZ2luID0gcGFyc2VJbnQoY29tcG91bmQuY3NzKCdwYWRkaW5nLWxlZnQnKSk7XHJcbi8vICAgICAgICB2YXIgdmVydGljYWxNYXJnaW4gPSBwYXJzZUludChjb21wb3VuZC5jc3MoJ3BhZGRpbmctdG9wJykpO1xyXG4vL1xyXG4vLyAgICAgICAgLy8gQWRqdXN0IHRoZSBwb3NpdGlvbnMgb2Ygbm9kZXMgd3J0IGl0cyBjb21wb3VuZFxyXG4vLyAgICAgICAgdGhpcy5hZGp1c3RMb2NhdGlvbnModGlsZWRQYWNrW2ldLCBjb21wb3VuZE5vZGUucmVjdC54LCBjb21wb3VuZE5vZGUucmVjdC55LCBob3Jpem9udGFsTWFyZ2luLCB2ZXJ0aWNhbE1hcmdpbik7XHJcbi8vXHJcbi8vICAgICAgICB2YXIgdGVtcGNoaWxkcmVuID0gY29tcG91bmQuc2NyYXRjaCgnc2JnblBkTGF5b3V0JykudGVtcGNoaWxkcmVuO1xyXG4vLyAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCB0ZW1wY2hpbGRyZW4ubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgICAgICB0ZW1wY2hpbGRyZW5baV0ucmVtb3ZlKCk7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICAvLyBSZW1vdmUgdGhlIGR1bW15IGNvbXBvdW5kXHJcbi8vICAgICAgICBjb21wb3VuZC5yZW1vdmUoKTtcclxuLy8gICAgfVxyXG4vL307XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgcGxhY2VzIGVhY2ggemVybyBkZWdyZWUgbWVtYmVyIHdydCBnaXZlbiAoeCx5KSBjb29yZGluYXRlcyAodG9wIGxlZnQpLiBcclxuICovXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuYWRqdXN0TG9jYXRpb25zID0gZnVuY3Rpb24gKG9yZ2FuaXphdGlvbiwgeCwgeSwgY29tcG91bmRIb3Jpem9udGFsTWFyZ2luLCBjb21wb3VuZFZlcnRpY2FsTWFyZ2luKSB7XHJcbi8vICAgIHggKz0gY29tcG91bmRIb3Jpem9udGFsTWFyZ2luO1xyXG4vLyAgICB5ICs9IGNvbXBvdW5kVmVydGljYWxNYXJnaW47XHJcbi8vXHJcbi8vICAgIHZhciBsZWZ0ID0geDtcclxuLy9cclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBvcmdhbml6YXRpb24ucm93cy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgdmFyIHJvdyA9IG9yZ2FuaXphdGlvbi5yb3dzW2ldO1xyXG4vLyAgICAgICAgeCA9IGxlZnQ7XHJcbi8vICAgICAgICB2YXIgbWF4SGVpZ2h0ID0gMDtcclxuLy9cclxuLy8gICAgICAgIGZvciAodmFyIGogPSAwOyBqIDwgcm93Lmxlbmd0aDsgaisrKSB7XHJcbi8vICAgICAgICAgICAgdmFyIGxub2RlID0gcm93W2pdO1xyXG4vLyAgICAgICAgICAgIHZhciBub2RlID0gdGhpcy5jeS5nZXRFbGVtZW50QnlJZChsbm9kZS5pZCk7XHJcbi8vXHJcbi8vICAgICAgICAgICAgbG5vZGUucmVjdC54ID0geDsvLyArIGxub2RlLnJlY3Qud2lkdGggLyAyO1xyXG4vLyAgICAgICAgICAgIGxub2RlLnJlY3QueSA9IHk7Ly8gKyBsbm9kZS5yZWN0LmhlaWdodCAvIDI7XHJcbi8vXHJcbi8vICAgICAgICAgICAgeCArPSBsbm9kZS5yZWN0LndpZHRoICsgb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nO1xyXG4vL1xyXG4vLyAgICAgICAgICAgIGlmIChsbm9kZS5yZWN0LmhlaWdodCA+IG1heEhlaWdodClcclxuLy8gICAgICAgICAgICAgICAgbWF4SGVpZ2h0ID0gbG5vZGUucmVjdC5oZWlnaHQ7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICB5ICs9IG1heEhlaWdodCArIG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vICAgIH1cclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS50aWxlQ29tcG91bmRNZW1iZXJzID0gZnVuY3Rpb24gKGNoaWxkR3JhcGhNYXApIHtcclxuLy8gICAgdmFyIHRpbGVkTWVtYmVyUGFjayA9IFtdO1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpZCBpbiBjaGlsZEdyYXBoTWFwKSB7XHJcbi8vICAgICAgICAvLyBBY2Nlc3MgbGF5b3V0SW5mbyBub2RlcyB0byBzZXQgdGhlIHdpZHRoIGFuZCBoZWlnaHQgb2YgY29tcG91bmRzXHJcbi8vICAgICAgICB2YXIgY29tcG91bmROb2RlID0gX1NiZ25QRExheW91dC5pZFRvTE5vZGVbaWRdO1xyXG4vL1xyXG4vLyAgICAgICAgdGlsZWRNZW1iZXJQYWNrW2lkXSA9IHRoaXMudGlsZU5vZGVzKGNoaWxkR3JhcGhNYXBbaWRdKTtcclxuLy9cclxuLy8gICAgICAgIGNvbXBvdW5kTm9kZS5yZWN0LndpZHRoID0gdGlsZWRNZW1iZXJQYWNrW2lkXS53aWR0aCArIDIwO1xyXG4vLyAgICAgICAgY29tcG91bmROb2RlLnJlY3QuaGVpZ2h0ID0gdGlsZWRNZW1iZXJQYWNrW2lkXS5oZWlnaHQgKyAyMDtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICByZXR1cm4gdGlsZWRNZW1iZXJQYWNrO1xyXG4vL307XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLnRpbGVOb2RlcyA9IGZ1bmN0aW9uIChub2Rlcykge1xyXG4vLyAgICB2YXIgc2VsZiA9IHRoaXM7XHJcbi8vICAgIHZhciB2ZXJ0aWNhbFBhZGRpbmcgPSB0eXBlb2Ygc2VsZi5vcHRpb25zLnRpbGluZ1BhZGRpbmdWZXJ0aWNhbCA9PT0gJ2Z1bmN0aW9uJyA/IHNlbGYub3B0aW9ucy50aWxpbmdQYWRkaW5nVmVydGljYWwuY2FsbCgpIDogc2VsZi5vcHRpb25zLnRpbGluZ1BhZGRpbmdWZXJ0aWNhbDtcclxuLy8gICAgdmFyIGhvcml6b250YWxQYWRkaW5nID0gdHlwZW9mIHNlbGYub3B0aW9ucy50aWxpbmdQYWRkaW5nSG9yaXpvbnRhbCA9PT0gJ2Z1bmN0aW9uJyA/IHNlbGYub3B0aW9ucy50aWxpbmdQYWRkaW5nSG9yaXpvbnRhbC5jYWxsKCkgOiBzZWxmLm9wdGlvbnMudGlsaW5nUGFkZGluZ0hvcml6b250YWw7XHJcbi8vICAgIHZhciBvcmdhbml6YXRpb24gPSB7XHJcbi8vICAgICAgICByb3dzOiBbXSxcclxuLy8gICAgICAgIHJvd1dpZHRoOiBbXSxcclxuLy8gICAgICAgIHJvd0hlaWdodDogW10sXHJcbi8vICAgICAgICB3aWR0aDogMjAsXHJcbi8vICAgICAgICBoZWlnaHQ6IDIwLFxyXG4vLyAgICAgICAgdmVydGljYWxQYWRkaW5nOiB2ZXJ0aWNhbFBhZGRpbmcsXHJcbi8vICAgICAgICBob3Jpem9udGFsUGFkZGluZzogaG9yaXpvbnRhbFBhZGRpbmdcclxuLy8gICAgfTtcclxuLy9cclxuLy8gICAgdmFyIGxheW91dE5vZGVzID0gW107XHJcbi8vXHJcbi8vICAgIC8vIEdldCBsYXlvdXQgbm9kZXNcclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgdmFyIG5vZGUgPSBub2Rlc1tpXTtcclxuLy8gICAgICAgIHZhciBsTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW25vZGUuaWQoKV07XHJcbi8vXHJcbi8vICAgICAgICBpZiAoIW5vZGUuc2NyYXRjaCgnY29zZUJpbGtlbnQnKSB8fCAhbm9kZS5zY3JhdGNoKCdjb3NlQmlsa2VudCcpLmR1bW15X3BhcmVudF9pZCkge1xyXG4vLyAgICAgICAgICAgIHZhciBvd25lciA9IGxOb2RlLm93bmVyO1xyXG4vLyAgICAgICAgICAgIG93bmVyLnJlbW92ZShsTm9kZSk7XHJcbi8vXHJcbi8vICAgICAgICAgICAgdGhpcy5nbS5yZXNldEFsbE5vZGVzKCk7XHJcbi8vICAgICAgICAgICAgdGhpcy5nbS5nZXRBbGxOb2RlcygpO1xyXG4vLyAgICAgICAgfVxyXG4vL1xyXG4vLyAgICAgICAgbGF5b3V0Tm9kZXMucHVzaChsTm9kZSk7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy8gU29ydCB0aGUgbm9kZXMgaW4gYXNjZW5kaW5nIG9yZGVyIG9mIHRoZWlyIGFyZWFzXHJcbi8vICAgIGxheW91dE5vZGVzLnNvcnQoZnVuY3Rpb24gKG4xLCBuMikge1xyXG4vLyAgICAgICAgaWYgKG4xLnJlY3Qud2lkdGggKiBuMS5yZWN0LmhlaWdodCA+IG4yLnJlY3Qud2lkdGggKiBuMi5yZWN0LmhlaWdodClcclxuLy8gICAgICAgICAgICByZXR1cm4gLTE7XHJcbi8vICAgICAgICBpZiAobjEucmVjdC53aWR0aCAqIG4xLnJlY3QuaGVpZ2h0IDwgbjIucmVjdC53aWR0aCAqIG4yLnJlY3QuaGVpZ2h0KVxyXG4vLyAgICAgICAgICAgIHJldHVybiAxO1xyXG4vLyAgICAgICAgcmV0dXJuIDA7XHJcbi8vICAgIH0pO1xyXG4vL1xyXG4vLyAgICAvLyBDcmVhdGUgdGhlIG9yZ2FuaXphdGlvbiAtPiB0aWxlIG1lbWJlcnNcclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsYXlvdXROb2Rlcy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgdmFyIGxOb2RlID0gbGF5b3V0Tm9kZXNbaV07XHJcbi8vXHJcbi8vICAgICAgICB2YXIgY3lOb2RlID0gdGhpcy5jeS5nZXRFbGVtZW50QnlJZChsTm9kZS5pZCkucGFyZW50KClbMF07XHJcbi8vICAgICAgICB2YXIgbWluV2lkdGggPSAwO1xyXG4vLyAgICAgICAgaWYgKGN5Tm9kZSkge1xyXG4vLyAgICAgICAgICAgIG1pbldpZHRoID0gcGFyc2VJbnQoY3lOb2RlLmNzcygncGFkZGluZy1sZWZ0JykpICsgcGFyc2VJbnQoY3lOb2RlLmNzcygncGFkZGluZy1yaWdodCcpKTtcclxuLy8gICAgICAgIH1cclxuLy9cclxuLy8gICAgICAgIGlmIChvcmdhbml6YXRpb24ucm93cy5sZW5ndGggPT0gMCkge1xyXG4vLyAgICAgICAgICAgIHRoaXMuaW5zZXJ0Tm9kZVRvUm93KG9yZ2FuaXphdGlvbiwgbE5vZGUsIDAsIG1pbldpZHRoKTtcclxuLy8gICAgICAgIH0gZWxzZSBpZiAodGhpcy5jYW5BZGRIb3Jpem9udGFsKG9yZ2FuaXphdGlvbiwgbE5vZGUucmVjdC53aWR0aCwgbE5vZGUucmVjdC5oZWlnaHQpKSB7XHJcbi8vICAgICAgICAgICAgdGhpcy5pbnNlcnROb2RlVG9Sb3cob3JnYW5pemF0aW9uLCBsTm9kZSwgdGhpcy5nZXRTaG9ydGVzdFJvd0luZGV4KG9yZ2FuaXphdGlvbiksIG1pbldpZHRoKTtcclxuLy8gICAgICAgIH0gZWxzZSB7XHJcbi8vICAgICAgICAgICAgdGhpcy5pbnNlcnROb2RlVG9Sb3cob3JnYW5pemF0aW9uLCBsTm9kZSwgb3JnYW5pemF0aW9uLnJvd3MubGVuZ3RoLCBtaW5XaWR0aCk7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICB0aGlzLnNoaWZ0VG9MYXN0Um93KG9yZ2FuaXphdGlvbik7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgcmV0dXJuIG9yZ2FuaXphdGlvbjtcclxuLy99O1xyXG4vL1xyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmluc2VydE5vZGVUb1JvdyA9IGZ1bmN0aW9uIChvcmdhbml6YXRpb24sIG5vZGUsIHJvd0luZGV4LCBtaW5XaWR0aCkge1xyXG4vLyAgICB2YXIgbWluQ29tcG91bmRTaXplID0gbWluV2lkdGg7XHJcbi8vXHJcbi8vICAgIC8vIEFkZCBuZXcgcm93IGlmIG5lZWRlZFxyXG4vLyAgICBpZiAocm93SW5kZXggPT0gb3JnYW5pemF0aW9uLnJvd3MubGVuZ3RoKSB7XHJcbi8vICAgICAgICB2YXIgc2Vjb25kRGltZW5zaW9uID0gW107XHJcbi8vXHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93cy5wdXNoKHNlY29uZERpbWVuc2lvbik7XHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93V2lkdGgucHVzaChtaW5Db21wb3VuZFNpemUpO1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLnJvd0hlaWdodC5wdXNoKDApO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIC8vIFVwZGF0ZSByb3cgd2lkdGhcclxuLy8gICAgdmFyIHcgPSBvcmdhbml6YXRpb24ucm93V2lkdGhbcm93SW5kZXhdICsgbm9kZS5yZWN0LndpZHRoO1xyXG4vL1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLnJvd3Nbcm93SW5kZXhdLmxlbmd0aCA+IDApIHtcclxuLy8gICAgICAgIHcgKz0gb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtyb3dJbmRleF0gPSB3O1xyXG4vLyAgICAvLyBVcGRhdGUgY29tcG91bmQgd2lkdGhcclxuLy8gICAgaWYgKG9yZ2FuaXphdGlvbi53aWR0aCA8IHcpIHtcclxuLy8gICAgICAgIG9yZ2FuaXphdGlvbi53aWR0aCA9IHc7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy8gVXBkYXRlIGhlaWdodFxyXG4vLyAgICB2YXIgaCA9IG5vZGUucmVjdC5oZWlnaHQ7XHJcbi8vICAgIGlmIChyb3dJbmRleCA+IDApXHJcbi8vICAgICAgICBoICs9IG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vXHJcbi8vICAgIHZhciBleHRyYUhlaWdodCA9IDA7XHJcbi8vICAgIGlmIChoID4gb3JnYW5pemF0aW9uLnJvd0hlaWdodFtyb3dJbmRleF0pIHtcclxuLy8gICAgICAgIGV4dHJhSGVpZ2h0ID0gb3JnYW5pemF0aW9uLnJvd0hlaWdodFtyb3dJbmRleF07XHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93SGVpZ2h0W3Jvd0luZGV4XSA9IGg7XHJcbi8vICAgICAgICBleHRyYUhlaWdodCA9IG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbcm93SW5kZXhdIC0gZXh0cmFIZWlnaHQ7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgb3JnYW5pemF0aW9uLmhlaWdodCArPSBleHRyYUhlaWdodDtcclxuLy9cclxuLy8gICAgLy8gSW5zZXJ0IG5vZGVcclxuLy8gICAgb3JnYW5pemF0aW9uLnJvd3Nbcm93SW5kZXhdLnB1c2gobm9kZSk7XHJcbi8vfTtcclxuLy9cclxuLy8vL1NjYW5zIHRoZSByb3dzIG9mIGFuIG9yZ2FuaXphdGlvbiBhbmQgcmV0dXJucyB0aGUgb25lIHdpdGggdGhlIG1pbiB3aWR0aFxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmdldFNob3J0ZXN0Um93SW5kZXggPSBmdW5jdGlvbiAob3JnYW5pemF0aW9uKSB7XHJcbi8vICAgIHZhciByID0gLTE7XHJcbi8vICAgIHZhciBtaW4gPSBOdW1iZXIuTUFYX1ZBTFVFO1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IG9yZ2FuaXphdGlvbi5yb3dzLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICBpZiAob3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldIDwgbWluKSB7XHJcbi8vICAgICAgICAgICAgciA9IGk7XHJcbi8vICAgICAgICAgICAgbWluID0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9XHJcbi8vICAgIHJldHVybiByO1xyXG4vL307XHJcbi8vXHJcbi8vLy9TY2FucyB0aGUgcm93cyBvZiBhbiBvcmdhbml6YXRpb24gYW5kIHJldHVybnMgdGhlIG9uZSB3aXRoIHRoZSBtYXggd2lkdGhcclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5nZXRMb25nZXN0Um93SW5kZXggPSBmdW5jdGlvbiAob3JnYW5pemF0aW9uKSB7XHJcbi8vICAgIHZhciByID0gLTE7XHJcbi8vICAgIHZhciBtYXggPSBOdW1iZXIuTUlOX1ZBTFVFO1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IG9yZ2FuaXphdGlvbi5yb3dzLmxlbmd0aDsgaSsrKSB7XHJcbi8vXHJcbi8vICAgICAgICBpZiAob3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldID4gbWF4KSB7XHJcbi8vICAgICAgICAgICAgciA9IGk7XHJcbi8vICAgICAgICAgICAgbWF4ID0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIHJldHVybiByO1xyXG4vL307XHJcbi8vXHJcbi8vLyoqXHJcbi8vICogVGhpcyBtZXRob2QgY2hlY2tzIHdoZXRoZXIgYWRkaW5nIGV4dHJhIHdpZHRoIHRvIHRoZSBvcmdhbml6YXRpb24gdmlvbGF0ZXNcclxuLy8gKiB0aGUgYXNwZWN0IHJhdGlvKDEpIG9yIG5vdC5cclxuLy8gKi9cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5jYW5BZGRIb3Jpem9udGFsID0gZnVuY3Rpb24gKG9yZ2FuaXphdGlvbiwgZXh0cmFXaWR0aCwgZXh0cmFIZWlnaHQpIHtcclxuLy9cclxuLy8gICAgdmFyIHNyaSA9IHRoaXMuZ2V0U2hvcnRlc3RSb3dJbmRleChvcmdhbml6YXRpb24pO1xyXG4vL1xyXG4vLyAgICBpZiAoc3JpIDwgMCkge1xyXG4vLyAgICAgICAgcmV0dXJuIHRydWU7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgdmFyIG1pbiA9IG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtzcmldO1xyXG4vL1xyXG4vLyAgICBpZiAobWluICsgb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nICsgZXh0cmFXaWR0aCA8PSBvcmdhbml6YXRpb24ud2lkdGgpXHJcbi8vICAgICAgICByZXR1cm4gdHJ1ZTtcclxuLy9cclxuLy8gICAgdmFyIGhEaWZmID0gMDtcclxuLy9cclxuLy8gICAgLy8gQWRkaW5nIHRvIGFuIGV4aXN0aW5nIHJvd1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLnJvd0hlaWdodFtzcmldIDwgZXh0cmFIZWlnaHQpIHtcclxuLy8gICAgICAgIGlmIChzcmkgPiAwKVxyXG4vLyAgICAgICAgICAgIGhEaWZmID0gZXh0cmFIZWlnaHQgKyBvcmdhbml6YXRpb24udmVydGljYWxQYWRkaW5nIC0gb3JnYW5pemF0aW9uLnJvd0hlaWdodFtzcmldO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIHZhciBhZGRfdG9fcm93X3JhdGlvO1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLndpZHRoIC0gbWluID49IGV4dHJhV2lkdGggKyBvcmdhbml6YXRpb24uaG9yaXpvbnRhbFBhZGRpbmcpIHtcclxuLy8gICAgICAgIGFkZF90b19yb3dfcmF0aW8gPSAob3JnYW5pemF0aW9uLmhlaWdodCArIGhEaWZmKSAvIChtaW4gKyBleHRyYVdpZHRoICsgb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nKTtcclxuLy8gICAgfSBlbHNlIHtcclxuLy8gICAgICAgIGFkZF90b19yb3dfcmF0aW8gPSAob3JnYW5pemF0aW9uLmhlaWdodCArIGhEaWZmKSAvIG9yZ2FuaXphdGlvbi53aWR0aDtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICAvLyBBZGRpbmcgYSBuZXcgcm93IGZvciB0aGlzIG5vZGVcclxuLy8gICAgaERpZmYgPSBleHRyYUhlaWdodCArIG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vICAgIHZhciBhZGRfbmV3X3Jvd19yYXRpbztcclxuLy8gICAgaWYgKG9yZ2FuaXphdGlvbi53aWR0aCA8IGV4dHJhV2lkdGgpIHtcclxuLy8gICAgICAgIGFkZF9uZXdfcm93X3JhdGlvID0gKG9yZ2FuaXphdGlvbi5oZWlnaHQgKyBoRGlmZikgLyBleHRyYVdpZHRoO1xyXG4vLyAgICB9IGVsc2Uge1xyXG4vLyAgICAgICAgYWRkX25ld19yb3dfcmF0aW8gPSAob3JnYW5pemF0aW9uLmhlaWdodCArIGhEaWZmKSAvIG9yZ2FuaXphdGlvbi53aWR0aDtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICBpZiAoYWRkX25ld19yb3dfcmF0aW8gPCAxKVxyXG4vLyAgICAgICAgYWRkX25ld19yb3dfcmF0aW8gPSAxIC8gYWRkX25ld19yb3dfcmF0aW87XHJcbi8vXHJcbi8vICAgIGlmIChhZGRfdG9fcm93X3JhdGlvIDwgMSlcclxuLy8gICAgICAgIGFkZF90b19yb3dfcmF0aW8gPSAxIC8gYWRkX3RvX3Jvd19yYXRpbztcclxuLy9cclxuLy8gICAgcmV0dXJuIGFkZF90b19yb3dfcmF0aW8gPCBhZGRfbmV3X3Jvd19yYXRpbztcclxuLy99O1xyXG4vL1xyXG4vL1xyXG4vLy8vSWYgbW92aW5nIHRoZSBsYXN0IG5vZGUgZnJvbSB0aGUgbG9uZ2VzdCByb3cgYW5kIGFkZGluZyBpdCB0byB0aGUgbGFzdFxyXG4vLy8vcm93IG1ha2VzIHRoZSBib3VuZGluZyBib3ggc21hbGxlciwgZG8gaXQuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuc2hpZnRUb0xhc3RSb3cgPSBmdW5jdGlvbiAob3JnYW5pemF0aW9uKSB7XHJcbi8vICAgIHZhciBsb25nZXN0ID0gdGhpcy5nZXRMb25nZXN0Um93SW5kZXgob3JnYW5pemF0aW9uKTtcclxuLy8gICAgdmFyIGxhc3QgPSBvcmdhbml6YXRpb24ucm93V2lkdGgubGVuZ3RoIC0gMTtcclxuLy8gICAgdmFyIHJvdyA9IG9yZ2FuaXphdGlvbi5yb3dzW2xvbmdlc3RdO1xyXG4vLyAgICB2YXIgbm9kZSA9IHJvd1tyb3cubGVuZ3RoIC0gMV07XHJcbi8vXHJcbi8vICAgIHZhciBkaWZmID0gbm9kZS53aWR0aCArIG9yZ2FuaXphdGlvbi5ob3Jpem9udGFsUGFkZGluZztcclxuLy9cclxuLy8gICAgLy8gQ2hlY2sgaWYgdGhlcmUgaXMgZW5vdWdoIHNwYWNlIG9uIHRoZSBsYXN0IHJvd1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLndpZHRoIC0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW2xhc3RdID4gZGlmZiAmJiBsb25nZXN0ICE9IGxhc3QpIHtcclxuLy8gICAgICAgIC8vIFJlbW92ZSB0aGUgbGFzdCBlbGVtZW50IG9mIHRoZSBsb25nZXN0IHJvd1xyXG4vLyAgICAgICAgcm93LnNwbGljZSgtMSwgMSk7XHJcbi8vXHJcbi8vICAgICAgICAvLyBQdXNoIGl0IHRvIHRoZSBsYXN0IHJvd1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLnJvd3NbbGFzdF0ucHVzaChub2RlKTtcclxuLy9cclxuLy8gICAgICAgIG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtsb25nZXN0XSA9IG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtsb25nZXN0XSAtIGRpZmY7XHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93V2lkdGhbbGFzdF0gPSBvcmdhbml6YXRpb24ucm93V2lkdGhbbGFzdF0gKyBkaWZmO1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLndpZHRoID0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW3RoaXMuZ2V0TG9uZ2VzdFJvd0luZGV4KG9yZ2FuaXphdGlvbildO1xyXG4vL1xyXG4vLyAgICAgICAgLy8gVXBkYXRlIGhlaWdodHMgb2YgdGhlIG9yZ2FuaXphdGlvblxyXG4vLyAgICAgICAgdmFyIG1heEhlaWdodCA9IE51bWJlci5NSU5fVkFMVUU7XHJcbi8vICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJvdy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgICAgIGlmIChyb3dbaV0uaGVpZ2h0ID4gbWF4SGVpZ2h0KVxyXG4vLyAgICAgICAgICAgICAgICBtYXhIZWlnaHQgPSByb3dbaV0uaGVpZ2h0O1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICAgICAgaWYgKGxvbmdlc3QgPiAwKVxyXG4vLyAgICAgICAgICAgIG1heEhlaWdodCArPSBvcmdhbml6YXRpb24udmVydGljYWxQYWRkaW5nO1xyXG4vL1xyXG4vLyAgICAgICAgdmFyIHByZXZUb3RhbCA9IG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbbG9uZ2VzdF0gKyBvcmdhbml6YXRpb24ucm93SGVpZ2h0W2xhc3RdO1xyXG4vL1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLnJvd0hlaWdodFtsb25nZXN0XSA9IG1heEhlaWdodDtcclxuLy8gICAgICAgIGlmIChvcmdhbml6YXRpb24ucm93SGVpZ2h0W2xhc3RdIDwgbm9kZS5oZWlnaHQgKyBvcmdhbml6YXRpb24udmVydGljYWxQYWRkaW5nKVxyXG4vLyAgICAgICAgICAgIG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbbGFzdF0gPSBub2RlLmhlaWdodCArIG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vXHJcbi8vICAgICAgICB2YXIgZmluYWxUb3RhbCA9IG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbbG9uZ2VzdF0gKyBvcmdhbml6YXRpb24ucm93SGVpZ2h0W2xhc3RdO1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLmhlaWdodCArPSAoZmluYWxUb3RhbCAtIHByZXZUb3RhbCk7XHJcbi8vXHJcbi8vICAgICAgICB0aGlzLnNoaWZ0VG9MYXN0Um93KG9yZ2FuaXphdGlvbik7XHJcbi8vICAgIH1cclxuLy99O1xyXG5cclxuLyoqXHJcbiAqIEBicmllZiA6IGNhbGxlZCBvbiBjb250aW51b3VzIGxheW91dHMgdG8gc3RvcCB0aGVtIGJlZm9yZSB0aGV5IGZpbmlzaFxyXG4gKi9cclxuX1NiZ25QRExheW91dC5wcm90b3R5cGUuc3RvcCA9IGZ1bmN0aW9uICgpIHtcclxuICAgIHRoaXMuc3RvcHBlZCA9IHRydWU7XHJcblxyXG4gICAgaWYgKHRoaXMudGhyZWFkKSB7XHJcbiAgICAgICAgdGhpcy50aHJlYWQuc3RvcCgpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMudHJpZ2dlcignbGF5b3V0c3RvcCcpO1xyXG5cclxuICAgIHJldHVybiB0aGlzOyAvLyBjaGFpbmluZ1xyXG59O1xyXG5cclxuX1NiZ25QRExheW91dC5wcm90b3R5cGUucHJvY2Vzc0NoaWxkcmVuTGlzdCA9IGZ1bmN0aW9uIChwYXJlbnQsIGNoaWxkcmVuKSB7XHJcbiAgICB2YXIgc2l6ZSA9IGNoaWxkcmVuLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZTsgaSsrKSB7XHJcbiAgICAgICAgdmFyIHRoZUNoaWxkID0gY2hpbGRyZW5baV07XHJcbiAgICAgICAgdGhpcy5vcHRpb25zLmVsZXMubm9kZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgdmFyIGNoaWxkcmVuX29mX2NoaWxkcmVuID0gdGhlQ2hpbGQuY2hpbGRyZW4oKTtcclxuICAgICAgICB2YXIgdGhlTm9kZTtcclxuXHJcbiAgICAgICAgaWYgKHRoZUNoaWxkLndpZHRoKCkgIT0gbnVsbFxyXG4gICAgICAgICAgICAgICAgJiYgdGhlQ2hpbGQuaGVpZ2h0KCkgIT0gbnVsbCkge1xyXG4gICAgICAgICAgICB0aGVOb2RlID0gcGFyZW50LmFkZChuZXcgQ29TRU5vZGUoX1NiZ25QRExheW91dC5sYXlvdXQuZ3JhcGhNYW5hZ2VyLFxyXG4gICAgICAgICAgICAgICAgICAgIG5ldyBQb2ludEQodGhlQ2hpbGQucG9zaXRpb24oJ3gnKSwgdGhlQ2hpbGQucG9zaXRpb24oJ3knKSksXHJcbiAgICAgICAgICAgICAgICAgICAgbmV3IERpbWVuc2lvbkQocGFyc2VGbG9hdCh0aGVDaGlsZC53aWR0aCgpKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHBhcnNlRmxvYXQodGhlQ2hpbGQuaGVpZ2h0KCkpKSkpO1xyXG4gICAgICAgIH0gZWxzZSB7XHJcbiAgICAgICAgICAgIHRoZU5vZGUgPSBwYXJlbnQuYWRkKG5ldyBDb1NFTm9kZSh0aGlzLmdyYXBoTWFuYWdlcikpO1xyXG4gICAgICAgIH1cclxuICAgICAgICB0aGVOb2RlLmlkID0gdGhlQ2hpbGQuZGF0YShcImlkXCIpO1xyXG4gICAgICAgIF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW3RoZUNoaWxkLmRhdGEoXCJpZFwiKV0gPSB0aGVOb2RlO1xyXG5cclxuICAgICAgICBpZiAoaXNOYU4odGhlTm9kZS5yZWN0LngpKSB7XHJcbiAgICAgICAgICAgIHRoZU5vZGUucmVjdC54ID0gMDtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIGlmIChpc05hTih0aGVOb2RlLnJlY3QueSkpIHtcclxuICAgICAgICAgICAgdGhlTm9kZS5yZWN0LnkgPSAwO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKGNoaWxkcmVuX29mX2NoaWxkcmVuICE9IG51bGwgJiYgY2hpbGRyZW5fb2ZfY2hpbGRyZW4ubGVuZ3RoID4gMCkge1xyXG4gICAgICAgICAgICB2YXIgdGhlTmV3R3JhcGg7XHJcbiAgICAgICAgICAgIHRoZU5ld0dyYXBoID0gX1NiZ25QRExheW91dC5sYXlvdXQuZ2V0R3JhcGhNYW5hZ2VyKCkuYWRkKF9TYmduUERMYXlvdXQubGF5b3V0Lm5ld0dyYXBoKCksIHRoZU5vZGUpO1xyXG4gICAgICAgICAgICB0aGlzLnByb2Nlc3NDaGlsZHJlbkxpc3QodGhlTmV3R3JhcGgsIGNoaWxkcmVuX29mX2NoaWxkcmVuKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbn07XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IGZ1bmN0aW9uIGdldChjeXRvc2NhcGUpIHtcclxuICAgIFRocmVhZCA9IGN5dG9zY2FwZS5UaHJlYWQ7XHJcblxyXG4gICAgcmV0dXJuIF9TYmduUERMYXlvdXQ7XHJcbn07XHJcbiIsIid1c2Ugc3RyaWN0JztcclxuXHJcbi8vIHJlZ2lzdGVycyB0aGUgZXh0ZW5zaW9uIG9uIGEgY3l0b3NjYXBlIGxpYiByZWZcclxudmFyIGdldExheW91dCA9IHJlcXVpcmUoJy4vTGF5b3V0Jyk7XHJcbi8vdmFyIGdldFV0aWxpdGllcyA9IHJlcXVpcmUoJy4vVXRpbGl0aWVzJyk7XHJcblxyXG52YXIgcmVnaXN0ZXIgPSBmdW5jdGlvbiggY3l0b3NjYXBlICl7XHJcbiAgdmFyIExheW91dCA9IGdldExheW91dCggY3l0b3NjYXBlICk7XHJcbiAgLy92YXIgVXRpbGl0aWVzID0gZ2V0VXRpbGl0aWVzICggY3l0b3NjYXBlICk7XHJcbiAgXHJcbiAgY3l0b3NjYXBlKCdsYXlvdXQnLCAnc2JnblBkTGF5b3V0JywgTGF5b3V0KTtcclxuICAvL2N5dG9zY2FwZSgnY29yZScsICd1dGlsaXRpZXMnLCBVdGlsaXRpZXMpO1xyXG59O1xyXG5cclxuaWYoIHR5cGVvZiBjeXRvc2NhcGUgIT09ICd1bmRlZmluZWQnICl7IC8vIGV4cG9zZSB0byBnbG9iYWwgY3l0b3NjYXBlIChpLmUuIHdpbmRvdy5jeXRvc2NhcGUpXHJcbiAgcmVnaXN0ZXIoIGN5dG9zY2FwZSApO1xyXG59XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IHJlZ2lzdGVyO1xyXG4iXX0=
