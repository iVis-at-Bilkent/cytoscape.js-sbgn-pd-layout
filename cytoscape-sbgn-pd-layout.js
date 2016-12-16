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

function SbgnPDEdge(source, target, vEdge, type) 
{
    CoSEEdge.call(this, source, target, vEdge);
    
    this.type = type;                // String (from LGraphObject)
    this.correspondingAngle = 0;     // int
    this.isProperlyOriented = false; // boolean
}

SbgnPDEdge.prototype = Object.create(CoSEEdge.prototype);
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
                    this.successRatio >= SbgnPDConstants.ROTATIONAL_FORCE_CONVERGENCE)
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
        this.rotateAProcess();
    }
            
    CoSELayout.prototype.moveNodes.call(this);
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
    this.enhancedRatio = totalProperEdges / this.totalEdgeCountToBeOriented;

    var numOfProcessNodes = this.processNodeList.length;
    for (var i; i<numOfProcessNodes; i++)
    {
        this.totalEffCount += this.processNodeList[i].effectorEdges.length;
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
    if (this.type != undefined)
    {
        return this.type.localeCompare(SbgnPDConstants.COMPLEX) === 0;
    }
    else
    {
        return -1;
    }
    
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
                    theNode = parent.add(new SbgnPDNode(gm_t,
                            new PointD(theChild.x, theChild.y),
                            new DimensionD(parseFloat(theChild.width),
                                    parseFloat(theChild.height))));
                } else {
                    theNode = parent.add(new SbgnPDNode(gm_t));
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
        /* TODO:
         * if (after.options.tile) {
            // Repopulate members
            // TODO:
            //after.repopulateZeroDegreeMembers(tiledZeroDegreeNodes);
            //after.repopulateCompounds(tiledMemberPack);
            after.options.eles.nodes().updateCompoundBounds();
        }*/

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
            theNode = parent.add(new SbgnPDNode(_SbgnPDLayout.layout.graphManager,
                    new PointD(theChild.position('x'), theChild.position('y')),
                    new DimensionD(parseFloat(theChild.width()),
                            parseFloat(theChild.height()))));
        } else {
            theNode = parent.add(new SbgnPDNode(this.graphManager));
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
//# sourceMappingURL=data:application/json;charset:utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyaWZ5L25vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJzcmMvTGF5b3V0L0NvU0VDb25zdGFudHMuanMiLCJzcmMvTGF5b3V0L0NvU0VFZGdlLmpzIiwic3JjL0xheW91dC9Db1NFR3JhcGguanMiLCJzcmMvTGF5b3V0L0NvU0VHcmFwaE1hbmFnZXIuanMiLCJzcmMvTGF5b3V0L0NvU0VMYXlvdXQuanMiLCJzcmMvTGF5b3V0L0NvU0VOb2RlLmpzIiwic3JjL0xheW91dC9Db21wYWN0aW9uLmpzIiwic3JjL0xheW91dC9EaW1lbnNpb25ELmpzIiwic3JjL0xheW91dC9GRExheW91dC5qcyIsInNyYy9MYXlvdXQvRkRMYXlvdXRDb25zdGFudHMuanMiLCJzcmMvTGF5b3V0L0ZETGF5b3V0RWRnZS5qcyIsInNyYy9MYXlvdXQvRkRMYXlvdXROb2RlLmpzIiwic3JjL0xheW91dC9IYXNoTWFwLmpzIiwic3JjL0xheW91dC9IYXNoU2V0LmpzIiwic3JjL0xheW91dC9JR2VvbWV0cnkuanMiLCJzcmMvTGF5b3V0L0lNYXRoLmpzIiwic3JjL0xheW91dC9JbnRlZ2VyLmpzIiwic3JjL0xheW91dC9MRWRnZS5qcyIsInNyYy9MYXlvdXQvTEdyYXBoLmpzIiwic3JjL0xheW91dC9MR3JhcGhNYW5hZ2VyLmpzIiwic3JjL0xheW91dC9MR3JhcGhPYmplY3QuanMiLCJzcmMvTGF5b3V0L0xOb2RlLmpzIiwic3JjL0xheW91dC9MYXlvdXQuanMiLCJzcmMvTGF5b3V0L0xheW91dENvbnN0YW50cy5qcyIsInNyYy9MYXlvdXQvTWVtYmVyUGFjay5qcyIsInNyYy9MYXlvdXQvT3JnYW5pemF0aW9uLmpzIiwic3JjL0xheW91dC9Qb2ludC5qcyIsInNyYy9MYXlvdXQvUG9pbnRELmpzIiwic3JjL0xheW91dC9Qb2x5b21pbm8uanMiLCJzcmMvTGF5b3V0L1BvbHlvbWlub1BhY2tpbmcuanMiLCJzcmMvTGF5b3V0L1BvbHlvbWlub1F1aWNrU29ydC5qcyIsInNyYy9MYXlvdXQvUmFuZG9tU2VlZC5qcyIsInNyYy9MYXlvdXQvUmVjdFByb2MuanMiLCJzcmMvTGF5b3V0L1JlY3RhbmdsZUQuanMiLCJzcmMvTGF5b3V0L1NiZ25QRENvbnN0YW50cy5qcyIsInNyYy9MYXlvdXQvU2JnblBERWRnZS5qcyIsInNyYy9MYXlvdXQvU2JnblBETGF5b3V0LmpzIiwic3JjL0xheW91dC9TYmduUEROb2RlLmpzIiwic3JjL0xheW91dC9TYmduUHJvY2Vzc05vZGUuanMiLCJzcmMvTGF5b3V0L1RyYW5zZm9ybS5qcyIsInNyYy9MYXlvdXQvVW5pcXVlSURHZW5lcmV0b3IuanMiLCJzcmMvTGF5b3V0L1Zpc2liaWxpdHlFZGdlLmpzIiwic3JjL0xheW91dC9WaXNpYmlsaXR5R3JhcGguanMiLCJzcmMvTGF5b3V0L2luZGV4LmpzIiwic3JjL2luZGV4LmpzIl0sIm5hbWVzIjpbXSwibWFwcGluZ3MiOiJBQUFBO0FDQUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDZkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDWkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDWkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDWkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3phQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDdkhBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDN01BO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzlCQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5V0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDOUJBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN2REE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzFaQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM5QkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNQQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3ZKQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDcGNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3RlQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDTEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUM3VkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3RwQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNsRkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN4RkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNoUUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN6RUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDaERBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2ZBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNqUUE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3phQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDWEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzlKQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2xJQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3RFQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ25FQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUN4aERBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQy9NQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ2ovQkE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQzNKQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7O0FDN0JBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBOztBQ3BDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUMzVEE7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTs7QUNqbENBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0E7QUFDQTtBQUNBO0FBQ0EiLCJmaWxlIjoiZ2VuZXJhdGVkLmpzIiwic291cmNlUm9vdCI6IiIsInNvdXJjZXNDb250ZW50IjpbIihmdW5jdGlvbiBlKHQsbixyKXtmdW5jdGlvbiBzKG8sdSl7aWYoIW5bb10pe2lmKCF0W29dKXt2YXIgYT10eXBlb2YgcmVxdWlyZT09XCJmdW5jdGlvblwiJiZyZXF1aXJlO2lmKCF1JiZhKXJldHVybiBhKG8sITApO2lmKGkpcmV0dXJuIGkobywhMCk7dmFyIGY9bmV3IEVycm9yKFwiQ2Fubm90IGZpbmQgbW9kdWxlICdcIitvK1wiJ1wiKTt0aHJvdyBmLmNvZGU9XCJNT0RVTEVfTk9UX0ZPVU5EXCIsZn12YXIgbD1uW29dPXtleHBvcnRzOnt9fTt0W29dWzBdLmNhbGwobC5leHBvcnRzLGZ1bmN0aW9uKGUpe3ZhciBuPXRbb11bMV1bZV07cmV0dXJuIHMobj9uOmUpfSxsLGwuZXhwb3J0cyxlLHQsbixyKX1yZXR1cm4gbltvXS5leHBvcnRzfXZhciBpPXR5cGVvZiByZXF1aXJlPT1cImZ1bmN0aW9uXCImJnJlcXVpcmU7Zm9yKHZhciBvPTA7bzxyLmxlbmd0aDtvKyspcyhyW29dKTtyZXR1cm4gc30pIiwidmFyIEZETGF5b3V0Q29uc3RhbnRzID0gcmVxdWlyZSgnLi9GRExheW91dENvbnN0YW50cycpO1xuXG5mdW5jdGlvbiBDb1NFQ29uc3RhbnRzKCkge1xufVxuXG4vL0NvU0VDb25zdGFudHMgaW5oZXJpdHMgc3RhdGljIHByb3BzIGluIEZETGF5b3V0Q29uc3RhbnRzXG5mb3IgKHZhciBwcm9wIGluIEZETGF5b3V0Q29uc3RhbnRzKSB7XG4gIENvU0VDb25zdGFudHNbcHJvcF0gPSBGRExheW91dENvbnN0YW50c1twcm9wXTtcbn1cblxuQ29TRUNvbnN0YW50cy5ERUZBVUxUX1VTRV9NVUxUSV9MRVZFTF9TQ0FMSU5HID0gZmFsc2U7XG5Db1NFQ29uc3RhbnRzLkRFRkFVTFRfUkFESUFMX1NFUEFSQVRJT04gPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIO1xuQ29TRUNvbnN0YW50cy5ERUZBVUxUX0NPTVBPTkVOVF9TRVBFUkFUSU9OID0gNjA7XG5cbm1vZHVsZS5leHBvcnRzID0gQ29TRUNvbnN0YW50cztcbiIsInZhciBGRExheW91dEVkZ2UgPSByZXF1aXJlKCcuL0ZETGF5b3V0RWRnZScpO1xuXG5mdW5jdGlvbiBDb1NFRWRnZShzb3VyY2UsIHRhcmdldCwgdkVkZ2UpIHtcbiAgRkRMYXlvdXRFZGdlLmNhbGwodGhpcywgc291cmNlLCB0YXJnZXQsIHZFZGdlKTtcbn1cblxuQ29TRUVkZ2UucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShGRExheW91dEVkZ2UucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gRkRMYXlvdXRFZGdlKSB7XG4gIENvU0VFZGdlW3Byb3BdID0gRkRMYXlvdXRFZGdlW3Byb3BdO1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IENvU0VFZGdlO1xuIiwidmFyIExHcmFwaCA9IHJlcXVpcmUoJy4vTEdyYXBoJyk7XG5cbmZ1bmN0aW9uIENvU0VHcmFwaChwYXJlbnQsIGdyYXBoTWdyLCB2R3JhcGgpIHtcbiAgTEdyYXBoLmNhbGwodGhpcywgcGFyZW50LCBncmFwaE1nciwgdkdyYXBoKTtcbn1cblxuQ29TRUdyYXBoLnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoTEdyYXBoLnByb3RvdHlwZSk7XG5mb3IgKHZhciBwcm9wIGluIExHcmFwaCkge1xuICBDb1NFR3JhcGhbcHJvcF0gPSBMR3JhcGhbcHJvcF07XG59XG5cbm1vZHVsZS5leHBvcnRzID0gQ29TRUdyYXBoO1xuIiwidmFyIExHcmFwaE1hbmFnZXIgPSByZXF1aXJlKCcuL0xHcmFwaE1hbmFnZXInKTtcblxuZnVuY3Rpb24gQ29TRUdyYXBoTWFuYWdlcihsYXlvdXQpIHtcbiAgTEdyYXBoTWFuYWdlci5jYWxsKHRoaXMsIGxheW91dCk7XG59XG5cbkNvU0VHcmFwaE1hbmFnZXIucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMR3JhcGhNYW5hZ2VyLnByb3RvdHlwZSk7XG5mb3IgKHZhciBwcm9wIGluIExHcmFwaE1hbmFnZXIpIHtcbiAgQ29TRUdyYXBoTWFuYWdlcltwcm9wXSA9IExHcmFwaE1hbmFnZXJbcHJvcF07XG59XG5cbm1vZHVsZS5leHBvcnRzID0gQ29TRUdyYXBoTWFuYWdlcjtcbiIsInZhciBGRExheW91dCA9IHJlcXVpcmUoJy4vRkRMYXlvdXQnKTtcbnZhciBDb1NFR3JhcGhNYW5hZ2VyID0gcmVxdWlyZSgnLi9Db1NFR3JhcGhNYW5hZ2VyJyk7XG52YXIgQ29TRUdyYXBoID0gcmVxdWlyZSgnLi9Db1NFR3JhcGgnKTtcbnZhciBDb1NFTm9kZSA9IHJlcXVpcmUoJy4vQ29TRU5vZGUnKTtcbnZhciBDb1NFRWRnZSA9IHJlcXVpcmUoJy4vQ29TRUVkZ2UnKTtcblxuZnVuY3Rpb24gQ29TRUxheW91dCgpIHtcbiAgRkRMYXlvdXQuY2FsbCh0aGlzKTtcbn1cblxuQ29TRUxheW91dC5wcm90b3R5cGUgPSBPYmplY3QuY3JlYXRlKEZETGF5b3V0LnByb3RvdHlwZSk7XG5cbmZvciAodmFyIHByb3AgaW4gRkRMYXlvdXQpIHtcbiAgQ29TRUxheW91dFtwcm9wXSA9IEZETGF5b3V0W3Byb3BdO1xufVxuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5uZXdHcmFwaE1hbmFnZXIgPSBmdW5jdGlvbiAoKSB7XG4gIHZhciBnbSA9IG5ldyBDb1NFR3JhcGhNYW5hZ2VyKHRoaXMpO1xuICB0aGlzLmdyYXBoTWFuYWdlciA9IGdtO1xuICByZXR1cm4gZ207XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5uZXdHcmFwaCA9IGZ1bmN0aW9uICh2R3JhcGgpIHtcbiAgcmV0dXJuIG5ldyBDb1NFR3JhcGgobnVsbCwgdGhpcy5ncmFwaE1hbmFnZXIsIHZHcmFwaCk7XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5uZXdOb2RlID0gZnVuY3Rpb24gKHZOb2RlKSB7XG4gIHJldHVybiBuZXcgQ29TRU5vZGUodGhpcy5ncmFwaE1hbmFnZXIsIHZOb2RlKTtcbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLm5ld0VkZ2UgPSBmdW5jdGlvbiAodkVkZ2UpIHtcbiAgcmV0dXJuIG5ldyBDb1NFRWRnZShudWxsLCBudWxsLCB2RWRnZSk7XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5pbml0UGFyYW1ldGVycyA9IGZ1bmN0aW9uICgpIHtcbiAgRkRMYXlvdXQucHJvdG90eXBlLmluaXRQYXJhbWV0ZXJzLmNhbGwodGhpcywgYXJndW1lbnRzKTtcbiAgaWYgKCF0aGlzLmlzU3ViTGF5b3V0KSB7XG4gICAgaWYgKENvU0VDb25zdGFudHMuREVGQVVMVF9FREdFX0xFTkdUSCA8IDEwKVxuICAgIHtcbiAgICAgIHRoaXMuaWRlYWxFZGdlTGVuZ3RoID0gMTA7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICB0aGlzLmlkZWFsRWRnZUxlbmd0aCA9IENvU0VDb25zdGFudHMuREVGQVVMVF9FREdFX0xFTkdUSDtcbiAgICB9XG5cbiAgICB0aGlzLnVzZVNtYXJ0SWRlYWxFZGdlTGVuZ3RoQ2FsY3VsYXRpb24gPVxuICAgICAgICAgICAgQ29TRUNvbnN0YW50cy5ERUZBVUxUX1VTRV9TTUFSVF9JREVBTF9FREdFX0xFTkdUSF9DQUxDVUxBVElPTjtcbiAgICB0aGlzLnNwcmluZ0NvbnN0YW50ID1cbiAgICAgICAgICAgIEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfU1BSSU5HX1NUUkVOR1RIO1xuICAgIHRoaXMucmVwdWxzaW9uQ29uc3RhbnQgPVxuICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9SRVBVTFNJT05fU1RSRU5HVEg7XG4gICAgdGhpcy5ncmF2aXR5Q29uc3RhbnQgPVxuICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1NUUkVOR1RIO1xuICAgIHRoaXMuY29tcG91bmRHcmF2aXR5Q29uc3RhbnQgPVxuICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1NUUkVOR1RIO1xuICAgIHRoaXMuZ3Jhdml0eVJhbmdlRmFjdG9yID1cbiAgICAgICAgICAgIEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9SQU5HRV9GQUNUT1I7XG4gICAgdGhpcy5jb21wb3VuZEdyYXZpdHlSYW5nZUZhY3RvciA9XG4gICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0NPTVBPVU5EX0dSQVZJVFlfUkFOR0VfRkFDVE9SO1xuICB9XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5sYXlvdXQgPSBmdW5jdGlvbiAoKSB7XG4gIHZhciBjcmVhdGVCZW5kc0FzTmVlZGVkID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ1JFQVRFX0JFTkRTX0FTX05FRURFRDtcbiAgaWYgKGNyZWF0ZUJlbmRzQXNOZWVkZWQpXG4gIHtcbiAgICB0aGlzLmNyZWF0ZUJlbmRwb2ludHMoKTtcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5yZXNldEFsbEVkZ2VzKCk7XG4gIH1cblxuICB0aGlzLmxldmVsID0gMDtcbiAgcmV0dXJuIHRoaXMuY2xhc3NpY0xheW91dCgpO1xufTtcblxuQ29TRUxheW91dC5wcm90b3R5cGUuY2xhc3NpY0xheW91dCA9IGZ1bmN0aW9uICgpIHtcbiAgdGhpcy5jYWxjdWxhdGVOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvblRvKCk7XG4gIHRoaXMuZ3JhcGhNYW5hZ2VyLmNhbGNMb3dlc3RDb21tb25BbmNlc3RvcnMoKTtcbiAgdGhpcy5ncmFwaE1hbmFnZXIuY2FsY0luY2x1c2lvblRyZWVEZXB0aHMoKTtcbiAgdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmNhbGNFc3RpbWF0ZWRTaXplKCk7XG4gIHRoaXMuY2FsY0lkZWFsRWRnZUxlbmd0aHMoKTtcbiAgaWYgKCF0aGlzLmluY3JlbWVudGFsKVxuICB7XG4gICAgdmFyIGZvcmVzdCA9IHRoaXMuZ2V0RmxhdEZvcmVzdCgpO1xuXG4gICAgLy8gVGhlIGdyYXBoIGFzc29jaWF0ZWQgd2l0aCB0aGlzIGxheW91dCBpcyBmbGF0IGFuZCBhIGZvcmVzdFxuICAgIGlmIChmb3Jlc3QubGVuZ3RoID4gMClcblxuICAgIHtcbiAgICAgIHRoaXMucG9zaXRpb25Ob2Rlc1JhZGlhbGx5KGZvcmVzdCk7XG4gICAgfVxuICAgIC8vIFRoZSBncmFwaCBhc3NvY2lhdGVkIHdpdGggdGhpcyBsYXlvdXQgaXMgbm90IGZsYXQgb3IgYSBmb3Jlc3RcbiAgICBlbHNlXG4gICAge1xuICAgICAgdGhpcy5wb3NpdGlvbk5vZGVzUmFuZG9tbHkoKTtcbiAgICB9XG4gIH1cblxuICB0aGlzLmluaXRTcHJpbmdFbWJlZGRlcigpO1xuICB0aGlzLnJ1blNwcmluZ0VtYmVkZGVyKCk7XG5cbiAgY29uc29sZS5sb2coXCJDbGFzc2ljIENvU0UgbGF5b3V0IGZpbmlzaGVkIGFmdGVyIFwiICtcbiAgICAgICAgICB0aGlzLnRvdGFsSXRlcmF0aW9ucyArIFwiIGl0ZXJhdGlvbnNcIik7XG5cbiAgcmV0dXJuIHRydWU7XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5ydW5TcHJpbmdFbWJlZGRlciA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGxhc3RGcmFtZSA9IG5ldyBEYXRlKCkuZ2V0VGltZSgpO1xuICB2YXIgaW5pdGlhbEFuaW1hdGlvblBlcmlvZCA9IDI1O1xuICB2YXIgYW5pbWF0aW9uUGVyaW9kID0gaW5pdGlhbEFuaW1hdGlvblBlcmlvZDtcbiAgZG9cbiAge1xuICAgIHRoaXMudG90YWxJdGVyYXRpb25zKys7XG5cbiAgICBpZiAodGhpcy50b3RhbEl0ZXJhdGlvbnMgJSBGRExheW91dENvbnN0YW50cy5DT05WRVJHRU5DRV9DSEVDS19QRVJJT0QgPT0gMClcbiAgICB7XG4gICAgICBpZiAodGhpcy5pc0NvbnZlcmdlZCgpKVxuICAgICAge1xuICAgICAgICBicmVhaztcbiAgICAgIH1cblxuICAgICAgdGhpcy5jb29saW5nRmFjdG9yID0gdGhpcy5pbml0aWFsQ29vbGluZ0ZhY3RvciAqXG4gICAgICAgICAgICAgICgodGhpcy5tYXhJdGVyYXRpb25zIC0gdGhpcy50b3RhbEl0ZXJhdGlvbnMpIC8gdGhpcy5tYXhJdGVyYXRpb25zKTtcbiAgICAgIGFuaW1hdGlvblBlcmlvZCA9IE1hdGguY2VpbChpbml0aWFsQW5pbWF0aW9uUGVyaW9kICogTWF0aC5zcXJ0KHRoaXMuY29vbGluZ0ZhY3RvcikpO1xuXG4gICAgfVxuICAgIHRoaXMudG90YWxEaXNwbGFjZW1lbnQgPSAwO1xuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnVwZGF0ZUJvdW5kcygpO1xuICAgIHRoaXMuY2FsY1NwcmluZ0ZvcmNlcygpO1xuICAgIHRoaXMuY2FsY1JlcHVsc2lvbkZvcmNlcygpO1xuICAgIHRoaXMuY2FsY0dyYXZpdGF0aW9uYWxGb3JjZXMoKTtcbiAgICB0aGlzLm1vdmVOb2RlcygpO1xuICAgIHRoaXMuYW5pbWF0ZSgpO1xuICAgIGlmIChGRExheW91dENvbnN0YW50cy5BTklNQVRFID09PSAnZHVyaW5nJyAmJiB0aGlzLnRvdGFsSXRlcmF0aW9ucyAlIGFuaW1hdGlvblBlcmlvZCA9PSAwKSB7XG4gICAgICBmb3IgKHZhciBpID0gMDsgaSA8IDFlNzsgaSsrKSB7XG4gICAgICAgIGlmICgobmV3IERhdGUoKS5nZXRUaW1lKCkgLSBsYXN0RnJhbWUpID4gMjUpIHtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuICAgICAgfVxuICAgICAgbGFzdEZyYW1lID0gbmV3IERhdGUoKS5nZXRUaW1lKCk7XG4gICAgICB2YXIgYWxsTm9kZXMgPSB0aGlzLmdyYXBoTWFuYWdlci5nZXRBbGxOb2RlcygpO1xuICAgICAgdmFyIHBEYXRhID0ge307XG4gICAgICBmb3IgKHZhciBpID0gMDsgaSA8IGFsbE5vZGVzLmxlbmd0aDsgaSsrKSB7XG4gICAgICAgIHZhciByZWN0ID0gYWxsTm9kZXNbaV0ucmVjdDtcbiAgICAgICAgdmFyIGlkID0gYWxsTm9kZXNbaV0uaWQ7XG4gICAgICAgIHBEYXRhW2lkXSA9IHtcbiAgICAgICAgICBpZDogaWQsXG4gICAgICAgICAgeDogcmVjdC5nZXRDZW50ZXJYKCksXG4gICAgICAgICAgeTogcmVjdC5nZXRDZW50ZXJZKCksXG4gICAgICAgICAgdzogcmVjdC53aWR0aCxcbiAgICAgICAgICBoOiByZWN0LmhlaWdodFxuICAgICAgICB9O1xuICAgICAgfVxuICAgICAgYnJvYWRjYXN0KHtwRGF0YTogcERhdGF9KTtcbiAgICB9XG4gIH1cbiAgd2hpbGUgKHRoaXMudG90YWxJdGVyYXRpb25zIDwgdGhpcy5tYXhJdGVyYXRpb25zKTtcblxuICB0aGlzLmdyYXBoTWFuYWdlci51cGRhdGVCb3VuZHMoKTtcbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLmNhbGN1bGF0ZU5vZGVzVG9BcHBseUdyYXZpdGF0aW9uVG8gPSBmdW5jdGlvbiAoKSB7XG4gIHZhciBub2RlTGlzdCA9IFtdO1xuICB2YXIgZ3JhcGg7XG5cbiAgdmFyIGdyYXBocyA9IHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEdyYXBocygpO1xuICB2YXIgc2l6ZSA9IGdyYXBocy5sZW5ndGg7XG4gIHZhciBpO1xuICBmb3IgKGkgPSAwOyBpIDwgc2l6ZTsgaSsrKVxuICB7XG4gICAgZ3JhcGggPSBncmFwaHNbaV07XG5cbiAgICBncmFwaC51cGRhdGVDb25uZWN0ZWQoKTtcblxuICAgIGlmICghZ3JhcGguaXNDb25uZWN0ZWQpXG4gICAge1xuICAgICAgbm9kZUxpc3QgPSBub2RlTGlzdC5jb25jYXQoZ3JhcGguZ2V0Tm9kZXMoKSk7XG4gICAgfVxuICB9XG5cbiAgdGhpcy5ncmFwaE1hbmFnZXIuc2V0QWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24obm9kZUxpc3QpO1xufTtcblxuQ29TRUxheW91dC5wcm90b3R5cGUuY3JlYXRlQmVuZHBvaW50cyA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGVkZ2VzID0gW107XG4gIGVkZ2VzID0gZWRnZXMuY29uY2F0KHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEFsbEVkZ2VzKCkpO1xuICB2YXIgdmlzaXRlZCA9IG5ldyBIYXNoU2V0KCk7XG4gIHZhciBpO1xuICBmb3IgKGkgPSAwOyBpIDwgZWRnZXMubGVuZ3RoOyBpKyspXG4gIHtcbiAgICB2YXIgZWRnZSA9IGVkZ2VzW2ldO1xuXG4gICAgaWYgKCF2aXNpdGVkLmNvbnRhaW5zKGVkZ2UpKVxuICAgIHtcbiAgICAgIHZhciBzb3VyY2UgPSBlZGdlLmdldFNvdXJjZSgpO1xuICAgICAgdmFyIHRhcmdldCA9IGVkZ2UuZ2V0VGFyZ2V0KCk7XG5cbiAgICAgIGlmIChzb3VyY2UgPT0gdGFyZ2V0KVxuICAgICAge1xuICAgICAgICBlZGdlLmdldEJlbmRwb2ludHMoKS5wdXNoKG5ldyBQb2ludEQoKSk7XG4gICAgICAgIGVkZ2UuZ2V0QmVuZHBvaW50cygpLnB1c2gobmV3IFBvaW50RCgpKTtcbiAgICAgICAgdGhpcy5jcmVhdGVEdW1teU5vZGVzRm9yQmVuZHBvaW50cyhlZGdlKTtcbiAgICAgICAgdmlzaXRlZC5hZGQoZWRnZSk7XG4gICAgICB9XG4gICAgICBlbHNlXG4gICAgICB7XG4gICAgICAgIHZhciBlZGdlTGlzdCA9IFtdO1xuXG4gICAgICAgIGVkZ2VMaXN0ID0gZWRnZUxpc3QuY29uY2F0KHNvdXJjZS5nZXRFZGdlTGlzdFRvTm9kZSh0YXJnZXQpKTtcbiAgICAgICAgZWRnZUxpc3QgPSBlZGdlTGlzdC5jb25jYXQodGFyZ2V0LmdldEVkZ2VMaXN0VG9Ob2RlKHNvdXJjZSkpO1xuXG4gICAgICAgIGlmICghdmlzaXRlZC5jb250YWlucyhlZGdlTGlzdFswXSkpXG4gICAgICAgIHtcbiAgICAgICAgICBpZiAoZWRnZUxpc3QubGVuZ3RoID4gMSlcbiAgICAgICAgICB7XG4gICAgICAgICAgICB2YXIgaztcbiAgICAgICAgICAgIGZvciAoayA9IDA7IGsgPCBlZGdlTGlzdC5sZW5ndGg7IGsrKylcbiAgICAgICAgICAgIHtcbiAgICAgICAgICAgICAgdmFyIG11bHRpRWRnZSA9IGVkZ2VMaXN0W2tdO1xuICAgICAgICAgICAgICBtdWx0aUVkZ2UuZ2V0QmVuZHBvaW50cygpLnB1c2gobmV3IFBvaW50RCgpKTtcbiAgICAgICAgICAgICAgdGhpcy5jcmVhdGVEdW1teU5vZGVzRm9yQmVuZHBvaW50cyhtdWx0aUVkZ2UpO1xuICAgICAgICAgICAgfVxuICAgICAgICAgIH1cbiAgICAgICAgICB2aXNpdGVkLmFkZEFsbChsaXN0KTtcbiAgICAgICAgfVxuICAgICAgfVxuICAgIH1cblxuICAgIGlmICh2aXNpdGVkLnNpemUoKSA9PSBlZGdlcy5sZW5ndGgpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9XG59O1xuXG5Db1NFTGF5b3V0LnByb3RvdHlwZS5wb3NpdGlvbk5vZGVzUmFkaWFsbHkgPSBmdW5jdGlvbiAoZm9yZXN0KSB7XG4gIC8vIFdlIHRpbGUgdGhlIHRyZWVzIHRvIGEgZ3JpZCByb3cgYnkgcm93OyBmaXJzdCB0cmVlIHN0YXJ0cyBhdCAoMCwwKVxuICB2YXIgY3VycmVudFN0YXJ0aW5nUG9pbnQgPSBuZXcgUG9pbnQoMCwgMCk7XG4gIHZhciBudW1iZXJPZkNvbHVtbnMgPSBNYXRoLmNlaWwoTWF0aC5zcXJ0KGZvcmVzdC5sZW5ndGgpKTtcbiAgdmFyIGhlaWdodCA9IDA7XG4gIHZhciBjdXJyZW50WSA9IDA7XG4gIHZhciBjdXJyZW50WCA9IDA7XG4gIHZhciBwb2ludCA9IG5ldyBQb2ludEQoMCwgMCk7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBmb3Jlc3QubGVuZ3RoOyBpKyspXG4gIHtcbiAgICBpZiAoaSAlIG51bWJlck9mQ29sdW1ucyA9PSAwKVxuICAgIHtcbiAgICAgIC8vIFN0YXJ0IG9mIGEgbmV3IHJvdywgbWFrZSB0aGUgeCBjb29yZGluYXRlIDAsIGluY3JlbWVudCB0aGVcbiAgICAgIC8vIHkgY29vcmRpbmF0ZSB3aXRoIHRoZSBtYXggaGVpZ2h0IG9mIHRoZSBwcmV2aW91cyByb3dcbiAgICAgIGN1cnJlbnRYID0gMDtcbiAgICAgIGN1cnJlbnRZID0gaGVpZ2h0O1xuXG4gICAgICBpZiAoaSAhPSAwKVxuICAgICAge1xuICAgICAgICBjdXJyZW50WSArPSBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfQ09NUE9ORU5UX1NFUEVSQVRJT047XG4gICAgICB9XG5cbiAgICAgIGhlaWdodCA9IDA7XG4gICAgfVxuXG4gICAgdmFyIHRyZWUgPSBmb3Jlc3RbaV07XG5cbiAgICAvLyBGaW5kIHRoZSBjZW50ZXIgb2YgdGhlIHRyZWVcbiAgICB2YXIgY2VudGVyTm9kZSA9IExheW91dC5maW5kQ2VudGVyT2ZUcmVlKHRyZWUpO1xuXG4gICAgLy8gU2V0IHRoZSBzdGFyaW5nIHBvaW50IG9mIHRoZSBuZXh0IHRyZWVcbiAgICBjdXJyZW50U3RhcnRpbmdQb2ludC54ID0gY3VycmVudFg7XG4gICAgY3VycmVudFN0YXJ0aW5nUG9pbnQueSA9IGN1cnJlbnRZO1xuXG4gICAgLy8gRG8gYSByYWRpYWwgbGF5b3V0IHN0YXJ0aW5nIHdpdGggdGhlIGNlbnRlclxuICAgIHBvaW50ID1cbiAgICAgICAgICAgIENvU0VMYXlvdXQucmFkaWFsTGF5b3V0KHRyZWUsIGNlbnRlck5vZGUsIGN1cnJlbnRTdGFydGluZ1BvaW50KTtcblxuICAgIGlmIChwb2ludC55ID4gaGVpZ2h0KVxuICAgIHtcbiAgICAgIGhlaWdodCA9IE1hdGguZmxvb3IocG9pbnQueSk7XG4gICAgfVxuXG4gICAgY3VycmVudFggPSBNYXRoLmZsb29yKHBvaW50LnggKyBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfQ09NUE9ORU5UX1NFUEVSQVRJT04pO1xuICB9XG5cbiAgdGhpcy50cmFuc2Zvcm0oXG4gICAgICAgICAgbmV3IFBvaW50RChMYXlvdXRDb25zdGFudHMuV09STERfQ0VOVEVSX1ggLSBwb2ludC54IC8gMixcbiAgICAgICAgICAgICAgICAgIExheW91dENvbnN0YW50cy5XT1JMRF9DRU5URVJfWSAtIHBvaW50LnkgLyAyKSk7XG59O1xuXG5Db1NFTGF5b3V0LnJhZGlhbExheW91dCA9IGZ1bmN0aW9uICh0cmVlLCBjZW50ZXJOb2RlLCBzdGFydGluZ1BvaW50KSB7XG4gIHZhciByYWRpYWxTZXAgPSBNYXRoLm1heCh0aGlzLm1heERpYWdvbmFsSW5UcmVlKHRyZWUpLFxuICAgICAgICAgIENvU0VDb25zdGFudHMuREVGQVVMVF9SQURJQUxfU0VQQVJBVElPTik7XG4gIENvU0VMYXlvdXQuYnJhbmNoUmFkaWFsTGF5b3V0KGNlbnRlck5vZGUsIG51bGwsIDAsIDM1OSwgMCwgcmFkaWFsU2VwKTtcbiAgdmFyIGJvdW5kcyA9IExHcmFwaC5jYWxjdWxhdGVCb3VuZHModHJlZSk7XG5cbiAgdmFyIHRyYW5zZm9ybSA9IG5ldyBUcmFuc2Zvcm0oKTtcbiAgdHJhbnNmb3JtLnNldERldmljZU9yZ1goYm91bmRzLmdldE1pblgoKSk7XG4gIHRyYW5zZm9ybS5zZXREZXZpY2VPcmdZKGJvdW5kcy5nZXRNaW5ZKCkpO1xuICB0cmFuc2Zvcm0uc2V0V29ybGRPcmdYKHN0YXJ0aW5nUG9pbnQueCk7XG4gIHRyYW5zZm9ybS5zZXRXb3JsZE9yZ1koc3RhcnRpbmdQb2ludC55KTtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHRyZWUubGVuZ3RoOyBpKyspXG4gIHtcbiAgICB2YXIgbm9kZSA9IHRyZWVbaV07XG4gICAgbm9kZS50cmFuc2Zvcm0odHJhbnNmb3JtKTtcbiAgfVxuXG4gIHZhciBib3R0b21SaWdodCA9XG4gICAgICAgICAgbmV3IFBvaW50RChib3VuZHMuZ2V0TWF4WCgpLCBib3VuZHMuZ2V0TWF4WSgpKTtcblxuICByZXR1cm4gdHJhbnNmb3JtLmludmVyc2VUcmFuc2Zvcm1Qb2ludChib3R0b21SaWdodCk7XG59O1xuXG5Db1NFTGF5b3V0LmJyYW5jaFJhZGlhbExheW91dCA9IGZ1bmN0aW9uIChub2RlLCBwYXJlbnRPZk5vZGUsIHN0YXJ0QW5nbGUsIGVuZEFuZ2xlLCBkaXN0YW5jZSwgcmFkaWFsU2VwYXJhdGlvbikge1xuICAvLyBGaXJzdCwgcG9zaXRpb24gdGhpcyBub2RlIGJ5IGZpbmRpbmcgaXRzIGFuZ2xlLlxuICB2YXIgaGFsZkludGVydmFsID0gKChlbmRBbmdsZSAtIHN0YXJ0QW5nbGUpICsgMSkgLyAyO1xuXG4gIGlmIChoYWxmSW50ZXJ2YWwgPCAwKVxuICB7XG4gICAgaGFsZkludGVydmFsICs9IDE4MDtcbiAgfVxuXG4gIHZhciBub2RlQW5nbGUgPSAoaGFsZkludGVydmFsICsgc3RhcnRBbmdsZSkgJSAzNjA7XG4gIHZhciB0ZXRhID0gKG5vZGVBbmdsZSAqIElHZW9tZXRyeS5UV09fUEkpIC8gMzYwO1xuXG4gIC8vIE1ha2UgcG9sYXIgdG8gamF2YSBjb3JkaW5hdGUgY29udmVyc2lvbi5cbiAgdmFyIGNvc190ZXRhID0gTWF0aC5jb3ModGV0YSk7XG4gIHZhciB4XyA9IGRpc3RhbmNlICogTWF0aC5jb3ModGV0YSk7XG4gIHZhciB5XyA9IGRpc3RhbmNlICogTWF0aC5zaW4odGV0YSk7XG5cbiAgbm9kZS5zZXRDZW50ZXIoeF8sIHlfKTtcblxuICAvLyBUcmF2ZXJzZSBhbGwgbmVpZ2hib3JzIG9mIHRoaXMgbm9kZSBhbmQgcmVjdXJzaXZlbHkgY2FsbCB0aGlzXG4gIC8vIGZ1bmN0aW9uLlxuICB2YXIgbmVpZ2hib3JFZGdlcyA9IFtdO1xuICBuZWlnaGJvckVkZ2VzID0gbmVpZ2hib3JFZGdlcy5jb25jYXQobm9kZS5nZXRFZGdlcygpKTtcbiAgdmFyIGNoaWxkQ291bnQgPSBuZWlnaGJvckVkZ2VzLmxlbmd0aDtcblxuICBpZiAocGFyZW50T2ZOb2RlICE9IG51bGwpXG4gIHtcbiAgICBjaGlsZENvdW50LS07XG4gIH1cblxuICB2YXIgYnJhbmNoQ291bnQgPSAwO1xuXG4gIHZhciBpbmNFZGdlc0NvdW50ID0gbmVpZ2hib3JFZGdlcy5sZW5ndGg7XG4gIHZhciBzdGFydEluZGV4O1xuXG4gIHZhciBlZGdlcyA9IG5vZGUuZ2V0RWRnZXNCZXR3ZWVuKHBhcmVudE9mTm9kZSk7XG5cbiAgLy8gSWYgdGhlcmUgYXJlIG11bHRpcGxlIGVkZ2VzLCBwcnVuZSB0aGVtIHVudGlsIHRoZXJlIHJlbWFpbnMgb25seSBvbmVcbiAgLy8gZWRnZS5cbiAgd2hpbGUgKGVkZ2VzLmxlbmd0aCA+IDEpXG4gIHtcbiAgICAvL25laWdoYm9yRWRnZXMucmVtb3ZlKGVkZ2VzLnJlbW92ZSgwKSk7XG4gICAgdmFyIHRlbXAgPSBlZGdlc1swXTtcbiAgICBlZGdlcy5zcGxpY2UoMCwgMSk7XG4gICAgdmFyIGluZGV4ID0gbmVpZ2hib3JFZGdlcy5pbmRleE9mKHRlbXApO1xuICAgIGlmIChpbmRleCA+PSAwKSB7XG4gICAgICBuZWlnaGJvckVkZ2VzLnNwbGljZShpbmRleCwgMSk7XG4gICAgfVxuICAgIGluY0VkZ2VzQ291bnQtLTtcbiAgICBjaGlsZENvdW50LS07XG4gIH1cblxuICBpZiAocGFyZW50T2ZOb2RlICE9IG51bGwpXG4gIHtcbiAgICAvL2Fzc2VydCBlZGdlcy5sZW5ndGggPT0gMTtcbiAgICBzdGFydEluZGV4ID0gKG5laWdoYm9yRWRnZXMuaW5kZXhPZihlZGdlc1swXSkgKyAxKSAlIGluY0VkZ2VzQ291bnQ7XG4gIH1cbiAgZWxzZVxuICB7XG4gICAgc3RhcnRJbmRleCA9IDA7XG4gIH1cblxuICB2YXIgc3RlcEFuZ2xlID0gTWF0aC5hYnMoZW5kQW5nbGUgLSBzdGFydEFuZ2xlKSAvIGNoaWxkQ291bnQ7XG5cbiAgZm9yICh2YXIgaSA9IHN0YXJ0SW5kZXg7XG4gICAgICAgICAgYnJhbmNoQ291bnQgIT0gY2hpbGRDb3VudDtcbiAgICAgICAgICBpID0gKCsraSkgJSBpbmNFZGdlc0NvdW50KVxuICB7XG4gICAgdmFyIGN1cnJlbnROZWlnaGJvciA9XG4gICAgICAgICAgICBuZWlnaGJvckVkZ2VzW2ldLmdldE90aGVyRW5kKG5vZGUpO1xuXG4gICAgLy8gRG9uJ3QgYmFjayB0cmF2ZXJzZSB0byByb290IG5vZGUgaW4gY3VycmVudCB0cmVlLlxuICAgIGlmIChjdXJyZW50TmVpZ2hib3IgPT0gcGFyZW50T2ZOb2RlKVxuICAgIHtcbiAgICAgIGNvbnRpbnVlO1xuICAgIH1cblxuICAgIHZhciBjaGlsZFN0YXJ0QW5nbGUgPVxuICAgICAgICAgICAgKHN0YXJ0QW5nbGUgKyBicmFuY2hDb3VudCAqIHN0ZXBBbmdsZSkgJSAzNjA7XG4gICAgdmFyIGNoaWxkRW5kQW5nbGUgPSAoY2hpbGRTdGFydEFuZ2xlICsgc3RlcEFuZ2xlKSAlIDM2MDtcblxuICAgIENvU0VMYXlvdXQuYnJhbmNoUmFkaWFsTGF5b3V0KGN1cnJlbnROZWlnaGJvcixcbiAgICAgICAgICAgIG5vZGUsXG4gICAgICAgICAgICBjaGlsZFN0YXJ0QW5nbGUsIGNoaWxkRW5kQW5nbGUsXG4gICAgICAgICAgICBkaXN0YW5jZSArIHJhZGlhbFNlcGFyYXRpb24sIHJhZGlhbFNlcGFyYXRpb24pO1xuXG4gICAgYnJhbmNoQ291bnQrKztcbiAgfVxufTtcblxuQ29TRUxheW91dC5tYXhEaWFnb25hbEluVHJlZSA9IGZ1bmN0aW9uICh0cmVlKSB7XG4gIHZhciBtYXhEaWFnb25hbCA9IEludGVnZXIuTUlOX1ZBTFVFO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgdHJlZS5sZW5ndGg7IGkrKylcbiAge1xuICAgIHZhciBub2RlID0gdHJlZVtpXTtcbiAgICB2YXIgZGlhZ29uYWwgPSBub2RlLmdldERpYWdvbmFsKCk7XG5cbiAgICBpZiAoZGlhZ29uYWwgPiBtYXhEaWFnb25hbClcbiAgICB7XG4gICAgICBtYXhEaWFnb25hbCA9IGRpYWdvbmFsO1xuICAgIH1cbiAgfVxuXG4gIHJldHVybiBtYXhEaWFnb25hbDtcbn07XG5cbkNvU0VMYXlvdXQucHJvdG90eXBlLmNhbGNSZXB1bHNpb25SYW5nZSA9IGZ1bmN0aW9uICgpIHtcbiAgLy8gZm9ybXVsYSBpcyAyIHggKGxldmVsICsgMSkgeCBpZGVhbEVkZ2VMZW5ndGhcbiAgcmV0dXJuICgyICogKHRoaXMubGV2ZWwgKyAxKSAqIHRoaXMuaWRlYWxFZGdlTGVuZ3RoKTtcbn07XG5cbm1vZHVsZS5leHBvcnRzID0gQ29TRUxheW91dDtcbiIsInZhciBGRExheW91dE5vZGUgPSByZXF1aXJlKCcuL0ZETGF5b3V0Tm9kZScpO1xuXG5mdW5jdGlvbiBDb1NFTm9kZShnbSwgbG9jLCBzaXplLCB2Tm9kZSkge1xuICBGRExheW91dE5vZGUuY2FsbCh0aGlzLCBnbSwgbG9jLCBzaXplLCB2Tm9kZSk7XG59XG5cblxuQ29TRU5vZGUucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShGRExheW91dE5vZGUucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gRkRMYXlvdXROb2RlKSB7XG4gIENvU0VOb2RlW3Byb3BdID0gRkRMYXlvdXROb2RlW3Byb3BdO1xufVxuXG5Db1NFTm9kZS5wcm90b3R5cGUubW92ZSA9IGZ1bmN0aW9uICgpXG57XG4gIHZhciBsYXlvdXQgPSB0aGlzLmdyYXBoTWFuYWdlci5nZXRMYXlvdXQoKTtcbiAgdGhpcy5kaXNwbGFjZW1lbnRYID0gbGF5b3V0LmNvb2xpbmdGYWN0b3IgKlxuICAgICAgICAgICh0aGlzLnNwcmluZ0ZvcmNlWCArIHRoaXMucmVwdWxzaW9uRm9yY2VYICsgdGhpcy5ncmF2aXRhdGlvbkZvcmNlWCk7XG4gIHRoaXMuZGlzcGxhY2VtZW50WSA9IGxheW91dC5jb29saW5nRmFjdG9yICpcbiAgICAgICAgICAodGhpcy5zcHJpbmdGb3JjZVkgKyB0aGlzLnJlcHVsc2lvbkZvcmNlWSArIHRoaXMuZ3Jhdml0YXRpb25Gb3JjZVkpO1xuXG5cbiAgaWYgKE1hdGguYWJzKHRoaXMuZGlzcGxhY2VtZW50WCkgPiBsYXlvdXQuY29vbGluZ0ZhY3RvciAqIGxheW91dC5tYXhOb2RlRGlzcGxhY2VtZW50KVxuICB7XG4gICAgdGhpcy5kaXNwbGFjZW1lbnRYID0gbGF5b3V0LmNvb2xpbmdGYWN0b3IgKiBsYXlvdXQubWF4Tm9kZURpc3BsYWNlbWVudCAqXG4gICAgICAgICAgICBJTWF0aC5zaWduKHRoaXMuZGlzcGxhY2VtZW50WCk7XG4gIH1cblxuICBpZiAoTWF0aC5hYnModGhpcy5kaXNwbGFjZW1lbnRZKSA+IGxheW91dC5jb29saW5nRmFjdG9yICogbGF5b3V0Lm1heE5vZGVEaXNwbGFjZW1lbnQpXG4gIHtcbiAgICB0aGlzLmRpc3BsYWNlbWVudFkgPSBsYXlvdXQuY29vbGluZ0ZhY3RvciAqIGxheW91dC5tYXhOb2RlRGlzcGxhY2VtZW50ICpcbiAgICAgICAgICAgIElNYXRoLnNpZ24odGhpcy5kaXNwbGFjZW1lbnRZKTtcbiAgfVxuXG4gIC8vIGEgc2ltcGxlIG5vZGUsIGp1c3QgbW92ZSBpdFxuICBpZiAodGhpcy5jaGlsZCA9PSBudWxsKVxuICB7XG4gICAgdGhpcy5tb3ZlQnkodGhpcy5kaXNwbGFjZW1lbnRYLCB0aGlzLmRpc3BsYWNlbWVudFkpO1xuICB9XG4gIC8vIGFuIGVtcHR5IGNvbXBvdW5kIG5vZGUsIGFnYWluIGp1c3QgbW92ZSBpdFxuICBlbHNlIGlmICh0aGlzLmNoaWxkLmdldE5vZGVzKCkubGVuZ3RoID09IDApXG4gIHtcbiAgICB0aGlzLm1vdmVCeSh0aGlzLmRpc3BsYWNlbWVudFgsIHRoaXMuZGlzcGxhY2VtZW50WSk7XG4gIH1cbiAgLy8gbm9uLWVtcHR5IGNvbXBvdW5kIG5vZGUsIHByb3BvZ2F0ZSBtb3ZlbWVudCB0byBjaGlsZHJlbiBhcyB3ZWxsXG4gIGVsc2VcbiAge1xuICAgIHRoaXMucHJvcG9nYXRlRGlzcGxhY2VtZW50VG9DaGlsZHJlbih0aGlzLmRpc3BsYWNlbWVudFgsXG4gICAgICAgICAgICB0aGlzLmRpc3BsYWNlbWVudFkpO1xuICB9XG5cbiAgbGF5b3V0LnRvdGFsRGlzcGxhY2VtZW50ICs9XG4gICAgICAgICAgTWF0aC5hYnModGhpcy5kaXNwbGFjZW1lbnRYKSArIE1hdGguYWJzKHRoaXMuZGlzcGxhY2VtZW50WSk7XG5cbiAgdGhpcy5zcHJpbmdGb3JjZVggPSAwO1xuICB0aGlzLnNwcmluZ0ZvcmNlWSA9IDA7XG4gIHRoaXMucmVwdWxzaW9uRm9yY2VYID0gMDtcbiAgdGhpcy5yZXB1bHNpb25Gb3JjZVkgPSAwO1xuICB0aGlzLmdyYXZpdGF0aW9uRm9yY2VYID0gMDtcbiAgdGhpcy5ncmF2aXRhdGlvbkZvcmNlWSA9IDA7XG4gIHRoaXMuZGlzcGxhY2VtZW50WCA9IDA7XG4gIHRoaXMuZGlzcGxhY2VtZW50WSA9IDA7XG59O1xuXG5Db1NFTm9kZS5wcm90b3R5cGUucHJvcG9nYXRlRGlzcGxhY2VtZW50VG9DaGlsZHJlbiA9IGZ1bmN0aW9uIChkWCwgZFkpXG57XG4gIHZhciBub2RlcyA9IHRoaXMuZ2V0Q2hpbGQoKS5nZXROb2RlcygpO1xuICB2YXIgbm9kZTtcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKylcbiAge1xuICAgIG5vZGUgPSBub2Rlc1tpXTtcbiAgICBpZiAobm9kZS5nZXRDaGlsZCgpID09IG51bGwpXG4gICAge1xuICAgICAgbm9kZS5tb3ZlQnkoZFgsIGRZKTtcbiAgICAgIG5vZGUuZGlzcGxhY2VtZW50WCArPSBkWDtcbiAgICAgIG5vZGUuZGlzcGxhY2VtZW50WSArPSBkWTtcbiAgICB9XG4gICAgZWxzZVxuICAgIHtcbiAgICAgIG5vZGUucHJvcG9nYXRlRGlzcGxhY2VtZW50VG9DaGlsZHJlbihkWCwgZFkpO1xuICAgIH1cbiAgfVxufTtcblxuQ29TRU5vZGUucHJvdG90eXBlLnNldFByZWQxID0gZnVuY3Rpb24gKHByZWQxKVxue1xuICB0aGlzLnByZWQxID0gcHJlZDE7XG59O1xuXG5Db1NFTm9kZS5wcm90b3R5cGUuZ2V0UHJlZDEgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gcHJlZDE7XG59O1xuXG5Db1NFTm9kZS5wcm90b3R5cGUuZ2V0UHJlZDIgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gcHJlZDI7XG59O1xuXG5Db1NFTm9kZS5wcm90b3R5cGUuc2V0TmV4dCA9IGZ1bmN0aW9uIChuZXh0KVxue1xuICB0aGlzLm5leHQgPSBuZXh0O1xufTtcblxuQ29TRU5vZGUucHJvdG90eXBlLmdldE5leHQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gbmV4dDtcbn07XG5cbkNvU0VOb2RlLnByb3RvdHlwZS5zZXRQcm9jZXNzZWQgPSBmdW5jdGlvbiAocHJvY2Vzc2VkKVxue1xuICB0aGlzLnByb2Nlc3NlZCA9IHByb2Nlc3NlZDtcbn07XG5cbkNvU0VOb2RlLnByb3RvdHlwZS5pc1Byb2Nlc3NlZCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiBwcm9jZXNzZWQ7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IENvU0VOb2RlO1xuIiwidmFyIFZpc2liaWxpdHlFZGdlID0gcmVxdWlyZSgnLi9WaXNpYmlsaXR5RWRnZScpO1xudmFyIFZpc2liaWxpdHlHcmFwaCA9IHJlcXVpcmUoJy4vVmlzaWJpbGl0eUdyYXBoJyk7XG52YXIgU2JnblBEQ29uc3RhbnRzID0gcmVxdWlyZSgnLi9TYmduUERDb25zdGFudHMnKTtcblxuZnVuY3Rpb24gQ29tcGFjdGlvbih2ZXJ0aWNlcykgXG57XG4gICAgdGhpcy5vcmRlcmVkTm9kZUxpc3QgPSBbXSAvKkFycmF5TGlzdDxTYmduUEROb2RlPigpKi87XG4gICAgdGhpcy52ZXJ0aWNlcyA9IHZlcnRpY2VzO1xuICAgIFxuICAgIHRoaXMudmlzR3JhcGggPSBudWxsO1xuICAgIHRoaXMuZGlyZWN0aW9uID0gbnVsbDtcbn1cblxuQ29tcGFjdGlvbi5wcm90b3R5cGUuQ29tcGFjdGlvbkRpcmVjdGlvbkVudW0gPSBcbntcbiAgICBWRVJUSUNBTCA6IDAsIFxuICAgIEhPUklaT05UQUwgOiAxXG59O1xuXG4vKipcbiogVHdvIHRpbWVzIGRvIHRoZSBmb2xsb3dpbmc6IChmaXJzdCBmb3IgdmVydGljYWwsIHNlY29uZCBob3Jpem9udGFsKSBGaXJzdFxuKiBjcmVhdGUgYSB2aXNpYmlsaXR5IGdyYXBoIGZvciB0aGUgZ2l2ZW4gZWxlbWVudHMuIFRoZSB2aXNpYmlsaXR5IGdyYXBoIGlzXG4qIGFsd2F5cyBhIERBRywgc28gcGVyZm9ybSBhIHRvcG9sb2dpY2FsIHNvcnQgb24gdGhlIGVsZW1lbnRzICh0aGUgbm9kZVxuKiB0aGF0IGhhcyBpbi1kZWdyZWUgMCBjb21lcyBmaXJzdCksIHBlcmZvcm0gY29tcGFjdGlvbi5cbiovXG5Db21wYWN0aW9uLnByb3RvdHlwZS5wZXJmb3JtID0gZnVuY3Rpb24gKClcbntcbiAgICB0aGlzLmFsZ29yaXRobUJvZHkodGhpcy5Db21wYWN0aW9uRGlyZWN0aW9uRW51bS5WRVJUSUNBTCk7XG4gICAgdGhpcy5yZW1vdmVWaXNpYmlsaXR5RWRnZXMoKTtcblxuICAgIHRoaXMuYWxnb3JpdGhtQm9keSh0aGlzLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpO1xuICAgIHRoaXMucmVtb3ZlVmlzaWJpbGl0eUVkZ2VzKCk7XG5cbn07XG5cbkNvbXBhY3Rpb24ucHJvdG90eXBlLnJlbW92ZVZpc2liaWxpdHlFZGdlcyA9IGZ1bmN0aW9uICgpXG57XG4gICAgdmFyIG51bU9mVmVydGljZXMgPSB0aGlzLnZlcnRpY2VzLmxlbmd0aDtcbiAgICBmb3IodmFyIGk9MDsgaTxudW1PZlZlcnRpY2VzOyBpKyspXG4gICAge1xuICAgICAgICB2YXIgc2Jnbk5vZGUgPSB0aGlzLnZlcnRpY2VzW2ldO1xuXG4gICAgICAgIGZvcih2YXIgaiA9IDA7IGogPCBzYmduTm9kZS5nZXRFZGdlcygpLmxlbmd0aDsgaisrKVxuICAgICAgICB7XG4gICAgICAgICAgICB2YXIgb2JqRWRnZSA9IHNiZ25Ob2RlLmdldEVkZ2VzKClbal07XG5cbiAgICAgICAgICAgIGlmKCBvYmpFZGdlIGluc3RhbmNlb2YgVmlzaWJpbGl0eUVkZ2UpXG4gICAgICAgICAgICB7XG4gICAgICAgICAgICAgICAgc2Jnbk5vZGUuZ2V0RWRnZXMoKS5zcGxpY2UoaSwgMSk7XG4gICAgICAgICAgICAgICAgaS0tO1xuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgfVx0XG59O1xuXG5Db21wYWN0aW9uLnByb3RvdHlwZS5hbGdvcml0aG1Cb2R5ID0gZnVuY3Rpb24gKGRpcmVjdGlvbilcbntcbiAgICB0aGlzLmRpcmVjdGlvbiA9IGRpcmVjdGlvbjtcbiAgICB0aGlzLnZpc0dyYXBoID0gbmV3IFZpc2liaWxpdHlHcmFwaChudWxsLCBudWxsLCBudWxsKTtcblxuICAgIC8vIGNvbnN0cnVjdCBhIHZpc2liaWxpdHkgZ3JhcGggZ2l2ZW4gdGhlIGRpcmVjdGlvbiBhbmQgdmVydGljZXNcbiAgICB0aGlzLnZpc0dyYXBoLmNvbnN0cnVjdCh0aGlzLmRpcmVjdGlvbiwgdmVydGljZXMpO1xuXG4gICAgaWYgKHRoaXMudmlzR3JhcGguZ2V0RWRnZXMoKS5sZW5ndGggPiAwKVxuICAgIHtcbiAgICAgICAgdGhpcy50b3BvbG9naWNhbGx5U29ydCgpO1xuICAgICAgICB0aGlzLmNvbXBhY3RFbGVtZW50cygpO1xuICAgIH1cblxuICAgIC8vIHBvc2l0aW9ucyBvZiB0aGUgdmVydGljZXMgaGFzIGNoYW5nZWQuIFVwZGF0ZSB0aGVtLlxuICAgIHRoaXMudmVydGljZXMgPSB0aGlzLnZpc0dyYXBoLmdldE5vZGVzKCk7XG59O1xuXG4vKipcbiogUGVyZm9ybSBhIERGUyBvbiB0aGUgZ2l2ZW4gZ3JhcGggbm9kZXMgYW5kIHRoZW4gb3V0cHV0IHRoZSBub2RlcyBpblxuKiByZXZlcnNlIG9yZGVyLlxuKi9cbkNvbXBhY3Rpb24ucHJvdG90eXBlLnRvcG9sb2dpY2FsbHlTb3J0ID0gZnVuY3Rpb24gKClcbntcbiAgICAvLyBlbnN1cmUgdGhhdCB0aGUgdmVydGljZXMgaGF2ZSBub3QgYmVlbiBtYXJrZWQgYXMgdmlzaXRlZC5cbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMudmlzR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGg7IGkrKylcbiAgICB7XG4gICAgICAgIHZhciBzID0gdGhpcy52aXNHcmFwaC5nZXROb2RlcygpW2ldO1xuICAgICAgICBzLnZpc2l0ZWQgPSBmYWxzZTtcbiAgICB9XG5cbiAgICAvLyBlbnN1cmUgdGhhdCB0aGUgbGlzdCBpcyBlbXB0eVxuICAgIHRoaXMub3JkZXJlZE5vZGVMaXN0ID0gW107XG4gICAgdGhpcy5ERlMoKTtcbiAgICB0aGlzLm9yZGVyZWROb2RlTGlzdCA9IHRoaXMucmV2ZXJzZUxpc3QodGhpcy5vcmRlcmVkTm9kZUxpc3QpO1xufTtcblxuQ29tcGFjdGlvbi5wcm90b3R5cGUuREZTID0gZnVuY3Rpb24gKClcbntcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMudmlzR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGg7IGkrKylcbiAgICB7XG4gICAgICAgIHZhciBzID0gdGhpcy52aXNHcmFwaC5nZXROb2RlcygpW2ldO1xuICAgICAgICBpZiAoIXMudmlzaXRlZClcbiAgICAgICAge1xuICAgICAgICAgICAgdGhpcy5ERlNfVmlzaXQocyk7XG4gICAgICAgIH1cbiAgICB9XG59O1xuXG5Db21wYWN0aW9uLnByb3RvdHlwZS5ERlNfVmlzaXQgPSBmdW5jdGlvbiAocylcbntcbiAgICB2YXIgbmVpZ2hib3JzID0gcy5nZXRDaGlsZHJlbk5laWdoYm9ycyhudWxsKTtcblxuICAgIGlmIChuZWlnaGJvcnMubGVuZ3RoID09PSAwKVxuICAgIHtcbiAgICAgICAgcy52aXNpdGVkID0gdHJ1ZTtcbiAgICAgICAgdGhpcy5vcmRlcmVkTm9kZUxpc3QucHVzaChzKTtcbiAgICAgICAgcmV0dXJuO1xuICAgIH1cblxuICAgIHZhciBudW1PZk5laWdoYm91cnMgPSBuZWlnaGJvcnMubGVuZ3RoO1xuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZk5laWdoYm91cnM7IGkrKylcbiAgICB7XG4gICAgICAgIGlmICghbmVpZ2hib3JzW2ldLnZpc2l0ZWQpXG4gICAgICAgIHtcbiAgICAgICAgICAgIHRoaXMuREZTX1Zpc2l0KG5laWdoYm9yc1tpXSk7XG4gICAgICAgIH1cbiAgICB9XG5cbiAgICBzLnZpc2l0ZWQgPSB0cnVlO1xuICAgIHRoaXMub3JkZXJlZE5vZGVMaXN0LnB1c2gocyk7XG59O1xuXG4vKipcbiogUmV2ZXJzZSB0aGUgZWxlbWVudCBvcmRlciBvZiBhIGdpdmVuIGxpc3RcbiovXG5Db21wYWN0aW9uLnByb3RvdHlwZS5yZXZlcnNlTGlzdCA9IGZ1bmN0aW9uIChvcmlnaW5hbExpc3QpXG57XG4gICAgdmFyIHJldmVyc2VPdXRwdXQgPSBbXTtcbiAgICBmb3IgKHZhciBpID0gb3JpZ2luYWxMaXN0Lmxlbmd0aCAtIDE7IGkgPj0gMDsgaS0tKVxuICAgIHtcbiAgICAgICAgcmV2ZXJzZU91dHB1dC5wdXNoKG9yaWdpbmFsTGlzdFtpXSk7XG4gICAgfVxuXG4gICAgcmV0dXJuIHJldmVyc2VPdXRwdXQ7XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgdmlzaXRzIHRoZSBsaXN0IHRoYXQgaXMgdGhlIHJlc3VsdCBvZiB0b3BvbG9naWNhbCBzb3J0LiBGb3JcbiogZWFjaCBub2RlIGluIHRoYXQgbGlzdCwgaXQgbG9va3MgZm9yIGl0cyBpbmNvbWluZyBlZGdlcyBhbmQgZmluZHMgdGhlXG4qIHNob3J0ZXN0IG9uZS4gVHJhbnNsYXRlcyB0aGUgbm9kZSB3cnQgdGhlIHNob3J0ZXN0IGVkZ2UuXG4qL1xuQ29tcGFjdGlvbi5wcm90b3R5cGUuY29tcGFjdEVsZW1lbnRzID0gZnVuY3Rpb24gKClcbntcbiAgICB2YXIgZGlzdGFuY2UgPSAwLjA7XG4gICAgXG4gICAgdmFyIG9yZGVyZWROb2RlTGlzdExlbmd0aCA9IHRoaXMub3JkZXJlZE5vZGVMaXN0Lmxlbmd0aDtcbiAgICBmb3IgKHZhciBpPTA7IGk8b3JkZXJlZE5vZGVMaXN0TGVuZ3RoOyBpKyspXG4gICAge1xuICAgICAgICB2YXIgc2JnblBETm9kZSA9IHRoaXMub3JkZXJlZE5vZGVMaXN0W2ldO1xuICAgICAgICBcbiAgICAgICAgLy8gZmluZCBzaG9ydGVzdCBpbmNvbWluZyBlZGdlXG4gICAgICAgIHZhciBlZGdlID0gdGhpcy52aXNHcmFwaC5maW5kU2hvcnRlc3RFZGdlKHNiZ25QRE5vZGUpO1xuXG4gICAgICAgIGlmIChlZGdlICE9IG51bGwpXG4gICAgICAgIHtcbiAgICAgICAgICAgIGRpc3RhbmNlID0gZWRnZS5nZXRMZW5ndGgoKTtcblxuICAgICAgICAgICAgaWYgKHRoaXMuZGlyZWN0aW9uID09PSB0aGlzLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKVxuICAgICAgICAgICAge1xuICAgICAgICAgICAgICAgIC8vIGJyaW5nIHRoZSBub2RlIGNsb3NlciB0byB0aGUgc291cmNlIG5vZGUgYW5kIHJlc3BlY3QgdGhlXG4gICAgICAgICAgICAgICAgLy8gYnVmZmVyLlxuICAgICAgICAgICAgICAgIGlmIChkaXN0YW5jZSA+IFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVIpXG4gICAgICAgICAgICAgICAge1xuICAgICAgICAgICAgICAgICAgICBzYmduUEROb2RlLnNldExvY2F0aW9uKFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHNiZ25QRE5vZGUuZ2V0TGVmdCgpLFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIChzYmduUEROb2RlLmdldFRvcCgpIC0gXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAoZGlzdGFuY2UgLSBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fVkVSVElDQUxfQlVGRkVSKSkpO1xuICAgICAgICAgICAgICAgIH1cbiAgICAgICAgICAgICAgICBlbHNlXG4gICAgICAgICAgICAgICAge1xuICAgICAgICAgICAgICAgICAgICBzYmduUEROb2RlLnNldExvY2F0aW9uKFxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHNiZ25QRE5vZGUuZ2V0TGVmdCgpLCBcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBlZGdlLmdldE90aGVyRW5kKHNiZ25QRE5vZGUpLmdldEJvdHRvbSgpXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVIpO1xuICAgICAgICAgICAgICAgIH1cbiAgICAgICAgICAgIH1cbiAgICAgICAgICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSB0aGlzLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXG4gICAgICAgICAgICB7XG4gICAgICAgICAgICAgICAgaWYgKGRpc3RhbmNlID4gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX0hPUklaT05UQUxfQlVGRkVSKVxuICAgICAgICAgICAgICAgIHtcbiAgICAgICAgICAgICAgICAgICAgc2JnblBETm9kZS5zZXRMb2NhdGlvbihcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAoc2JnblBETm9kZS5nZXRMZWZ0KCkgLSAoZGlzdGFuY2UgLSBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fSE9SSVpPTlRBTF9CVUZGRVIpKSxcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBzYmduUEROb2RlLmdldFRvcCgpKTtcbiAgICAgICAgICAgICAgICB9XG4gICAgICAgICAgICAgICAgZWxzZVxuICAgICAgICAgICAgICAgIHtcbiAgICAgICAgICAgICAgICAgICAgc2JnblBETm9kZS5zZXRMb2NhdGlvbihcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBlZGdlLmdldE90aGVyRW5kKHNiZ25QRE5vZGUpLmdldFJpZ2h0KClcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgKyBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fSE9SSVpPTlRBTF9CVUZGRVIsXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgc2JnblBETm9kZS5nZXRUb3AoKSk7XG4gICAgICAgICAgICAgICAgfVxuICAgICAgICAgICAgfVxuICAgICAgICB9XG4gICAgfVxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBDb21wYWN0aW9uO1xuXG5cbiIsImZ1bmN0aW9uIERpbWVuc2lvbkQod2lkdGgsIGhlaWdodCkge1xuICB0aGlzLndpZHRoID0gMDtcbiAgdGhpcy5oZWlnaHQgPSAwO1xuICBpZiAod2lkdGggIT09IG51bGwgJiYgaGVpZ2h0ICE9PSBudWxsKSB7XG4gICAgdGhpcy5oZWlnaHQgPSBoZWlnaHQ7XG4gICAgdGhpcy53aWR0aCA9IHdpZHRoO1xuICB9XG59XG5cbkRpbWVuc2lvbkQucHJvdG90eXBlLmdldFdpZHRoID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMud2lkdGg7XG59O1xuXG5EaW1lbnNpb25ELnByb3RvdHlwZS5zZXRXaWR0aCA9IGZ1bmN0aW9uICh3aWR0aClcbntcbiAgdGhpcy53aWR0aCA9IHdpZHRoO1xufTtcblxuRGltZW5zaW9uRC5wcm90b3R5cGUuZ2V0SGVpZ2h0ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuaGVpZ2h0O1xufTtcblxuRGltZW5zaW9uRC5wcm90b3R5cGUuc2V0SGVpZ2h0ID0gZnVuY3Rpb24gKGhlaWdodClcbntcbiAgdGhpcy5oZWlnaHQgPSBoZWlnaHQ7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IERpbWVuc2lvbkQ7XG4iLCJ2YXIgTGF5b3V0ID0gcmVxdWlyZSgnLi9MYXlvdXQnKTtcbnZhciBGRExheW91dENvbnN0YW50cyA9IHJlcXVpcmUoJy4vRkRMYXlvdXRDb25zdGFudHMnKTtcblxuZnVuY3Rpb24gRkRMYXlvdXQoKSB7XG4gIExheW91dC5jYWxsKHRoaXMpO1xuXG4gIHRoaXMudXNlU21hcnRJZGVhbEVkZ2VMZW5ndGhDYWxjdWxhdGlvbiA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfVVNFX1NNQVJUX0lERUFMX0VER0VfTEVOR1RIX0NBTENVTEFUSU9OO1xuICB0aGlzLmlkZWFsRWRnZUxlbmd0aCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEg7XG4gIHRoaXMuc3ByaW5nQ29uc3RhbnQgPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX1NQUklOR19TVFJFTkdUSDtcbiAgdGhpcy5yZXB1bHNpb25Db25zdGFudCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfUkVQVUxTSU9OX1NUUkVOR1RIO1xuICB0aGlzLmdyYXZpdHlDb25zdGFudCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9TVFJFTkdUSDtcbiAgdGhpcy5jb21wb3VuZEdyYXZpdHlDb25zdGFudCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9TVFJFTkdUSDtcbiAgdGhpcy5ncmF2aXR5UmFuZ2VGYWN0b3IgPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0dSQVZJVFlfUkFOR0VfRkFDVE9SO1xuICB0aGlzLmNvbXBvdW5kR3Jhdml0eVJhbmdlRmFjdG9yID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUjtcbiAgdGhpcy5kaXNwbGFjZW1lbnRUaHJlc2hvbGRQZXJOb2RlID0gKDMuMCAqIEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEgpIC8gMTAwO1xuICB0aGlzLmNvb2xpbmdGYWN0b3IgPSAxLjA7XG4gIHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3IgPSAxLjA7XG4gIHRoaXMudG90YWxEaXNwbGFjZW1lbnQgPSAwLjA7XG4gIHRoaXMub2xkVG90YWxEaXNwbGFjZW1lbnQgPSAwLjA7XG4gIHRoaXMubWF4SXRlcmF0aW9ucyA9IEZETGF5b3V0Q29uc3RhbnRzLk1BWF9JVEVSQVRJT05TO1xufVxuXG5GRExheW91dC5wcm90b3R5cGUgPSBPYmplY3QuY3JlYXRlKExheW91dC5wcm90b3R5cGUpO1xuXG5mb3IgKHZhciBwcm9wIGluIExheW91dCkge1xuICBGRExheW91dFtwcm9wXSA9IExheW91dFtwcm9wXTtcbn1cblxuRkRMYXlvdXQucHJvdG90eXBlLmluaXRQYXJhbWV0ZXJzID0gZnVuY3Rpb24gKCkge1xuICBMYXlvdXQucHJvdG90eXBlLmluaXRQYXJhbWV0ZXJzLmNhbGwodGhpcywgYXJndW1lbnRzKTtcblxuICBpZiAodGhpcy5sYXlvdXRRdWFsaXR5ID09IExheW91dENvbnN0YW50cy5EUkFGVF9RVUFMSVRZKVxuICB7XG4gICAgdGhpcy5kaXNwbGFjZW1lbnRUaHJlc2hvbGRQZXJOb2RlICs9IDAuMzA7XG4gICAgdGhpcy5tYXhJdGVyYXRpb25zICo9IDAuODtcbiAgfVxuICBlbHNlIGlmICh0aGlzLmxheW91dFF1YWxpdHkgPT0gTGF5b3V0Q29uc3RhbnRzLlBST09GX1FVQUxJVFkpXG4gIHtcbiAgICB0aGlzLmRpc3BsYWNlbWVudFRocmVzaG9sZFBlck5vZGUgLT0gMC4zMDtcbiAgICB0aGlzLm1heEl0ZXJhdGlvbnMgKj0gMS4yO1xuICB9XG5cbiAgdGhpcy50b3RhbEl0ZXJhdGlvbnMgPSAwO1xuICB0aGlzLm5vdEFuaW1hdGVkSXRlcmF0aW9ucyA9IDA7XG5cbi8vICAgIHRoaXMudXNlRlJHcmlkVmFyaWFudCA9IGxheW91dE9wdGlvbnNQYWNrLnNtYXJ0UmVwdWxzaW9uUmFuZ2VDYWxjO1xufTtcblxuRkRMYXlvdXQucHJvdG90eXBlLmNhbGNJZGVhbEVkZ2VMZW5ndGhzID0gZnVuY3Rpb24gKCkge1xuICB2YXIgZWRnZTtcbiAgdmFyIGxjYURlcHRoO1xuICB2YXIgc291cmNlO1xuICB2YXIgdGFyZ2V0O1xuICB2YXIgc2l6ZU9mU291cmNlSW5MY2E7XG4gIHZhciBzaXplT2ZUYXJnZXRJbkxjYTtcblxuICB2YXIgYWxsRWRnZXMgPSB0aGlzLmdldEdyYXBoTWFuYWdlcigpLmdldEFsbEVkZ2VzKCk7XG4gIGZvciAodmFyIGkgPSAwOyBpIDwgYWxsRWRnZXMubGVuZ3RoOyBpKyspXG4gIHtcbiAgICBlZGdlID0gYWxsRWRnZXNbaV07XG5cbiAgICBlZGdlLmlkZWFsTGVuZ3RoID0gdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XG5cbiAgICBpZiAoZWRnZS5pc0ludGVyR3JhcGgpXG4gICAge1xuICAgICAgc291cmNlID0gZWRnZS5nZXRTb3VyY2UoKTtcbiAgICAgIHRhcmdldCA9IGVkZ2UuZ2V0VGFyZ2V0KCk7XG5cbiAgICAgIHNpemVPZlNvdXJjZUluTGNhID0gZWRnZS5nZXRTb3VyY2VJbkxjYSgpLmdldEVzdGltYXRlZFNpemUoKTtcbiAgICAgIHNpemVPZlRhcmdldEluTGNhID0gZWRnZS5nZXRUYXJnZXRJbkxjYSgpLmdldEVzdGltYXRlZFNpemUoKTtcblxuICAgICAgaWYgKHRoaXMudXNlU21hcnRJZGVhbEVkZ2VMZW5ndGhDYWxjdWxhdGlvbilcbiAgICAgIHtcbiAgICAgICAgZWRnZS5pZGVhbExlbmd0aCArPSBzaXplT2ZTb3VyY2VJbkxjYSArIHNpemVPZlRhcmdldEluTGNhIC1cbiAgICAgICAgICAgICAgICAyICogTGF5b3V0Q29uc3RhbnRzLlNJTVBMRV9OT0RFX1NJWkU7XG4gICAgICB9XG5cbiAgICAgIGxjYURlcHRoID0gZWRnZS5nZXRMY2EoKS5nZXRJbmNsdXNpb25UcmVlRGVwdGgoKTtcblxuICAgICAgZWRnZS5pZGVhbExlbmd0aCArPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIICpcbiAgICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuUEVSX0xFVkVMX0lERUFMX0VER0VfTEVOR1RIX0ZBQ1RPUiAqXG4gICAgICAgICAgICAgIChzb3VyY2UuZ2V0SW5jbHVzaW9uVHJlZURlcHRoKCkgK1xuICAgICAgICAgICAgICAgICAgICAgIHRhcmdldC5nZXRJbmNsdXNpb25UcmVlRGVwdGgoKSAtIDIgKiBsY2FEZXB0aCk7XG4gICAgfVxuICB9XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuaW5pdFNwcmluZ0VtYmVkZGVyID0gZnVuY3Rpb24gKCkge1xuXG4gIGlmICh0aGlzLmluY3JlbWVudGFsKVxuICB7XG4gICAgdGhpcy5jb29saW5nRmFjdG9yID0gMC44O1xuICAgIHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3IgPSAwLjg7XG4gICAgdGhpcy5tYXhOb2RlRGlzcGxhY2VtZW50ID1cbiAgICAgICAgICAgIEZETGF5b3V0Q29uc3RhbnRzLk1BWF9OT0RFX0RJU1BMQUNFTUVOVF9JTkNSRU1FTlRBTDtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICB0aGlzLmNvb2xpbmdGYWN0b3IgPSAxLjA7XG4gICAgdGhpcy5pbml0aWFsQ29vbGluZ0ZhY3RvciA9IDEuMDtcbiAgICB0aGlzLm1heE5vZGVEaXNwbGFjZW1lbnQgPVxuICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuTUFYX05PREVfRElTUExBQ0VNRU5UO1xuICB9XG5cbiAgdGhpcy5tYXhJdGVyYXRpb25zID1cbiAgICAgICAgICBNYXRoLm1heCh0aGlzLmdldEFsbE5vZGVzKCkubGVuZ3RoICogNSwgdGhpcy5tYXhJdGVyYXRpb25zKTtcblxuICB0aGlzLnRvdGFsRGlzcGxhY2VtZW50VGhyZXNob2xkID1cbiAgICAgICAgICB0aGlzLmRpc3BsYWNlbWVudFRocmVzaG9sZFBlck5vZGUgKiB0aGlzLmdldEFsbE5vZGVzKCkubGVuZ3RoO1xuXG4gIHRoaXMucmVwdWxzaW9uUmFuZ2UgPSB0aGlzLmNhbGNSZXB1bHNpb25SYW5nZSgpO1xufTtcblxuRkRMYXlvdXQucHJvdG90eXBlLmNhbGNTcHJpbmdGb3JjZXMgPSBmdW5jdGlvbiAoKSB7XG4gIHZhciBsRWRnZXMgPSB0aGlzLmdldEFsbEVkZ2VzKCk7XG4gIHZhciBlZGdlO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgbEVkZ2VzLmxlbmd0aDsgaSsrKVxuICB7XG4gICAgZWRnZSA9IGxFZGdlc1tpXTtcblxuICAgIHRoaXMuY2FsY1NwcmluZ0ZvcmNlKGVkZ2UsIGVkZ2UuaWRlYWxMZW5ndGgpO1xuICB9XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuY2FsY1JlcHVsc2lvbkZvcmNlcyA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGksIGo7XG4gIHZhciBub2RlQSwgbm9kZUI7XG4gIHZhciBsTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCk7XG5cbiAgZm9yIChpID0gMDsgaSA8IGxOb2Rlcy5sZW5ndGg7IGkrKylcbiAge1xuICAgIG5vZGVBID0gbE5vZGVzW2ldO1xuXG4gICAgZm9yIChqID0gaSArIDE7IGogPCBsTm9kZXMubGVuZ3RoOyBqKyspXG4gICAge1xuICAgICAgbm9kZUIgPSBsTm9kZXNbal07XG5cbiAgICAgIC8vIElmIGJvdGggbm9kZXMgYXJlIG5vdCBtZW1iZXJzIG9mIHRoZSBzYW1lIGdyYXBoLCBza2lwLlxuICAgICAgaWYgKG5vZGVBLmdldE93bmVyKCkgIT0gbm9kZUIuZ2V0T3duZXIoKSlcbiAgICAgIHtcbiAgICAgICAgY29udGludWU7XG4gICAgICB9XG5cbiAgICAgIHRoaXMuY2FsY1JlcHVsc2lvbkZvcmNlKG5vZGVBLCBub2RlQik7XG4gICAgfVxuICB9XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuY2FsY0dyYXZpdGF0aW9uYWxGb3JjZXMgPSBmdW5jdGlvbiAoKSB7XG4gIHZhciBub2RlO1xuICB2YXIgbE5vZGVzID0gdGhpcy5nZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbigpO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgbE5vZGVzLmxlbmd0aDsgaSsrKVxuICB7XG4gICAgbm9kZSA9IGxOb2Rlc1tpXTtcbiAgICB0aGlzLmNhbGNHcmF2aXRhdGlvbmFsRm9yY2Uobm9kZSk7XG4gIH1cbn07XG5cbkZETGF5b3V0LnByb3RvdHlwZS5tb3ZlTm9kZXMgPSBmdW5jdGlvbiAoKSB7XG4gIHZhciBsTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCk7XG4gIHZhciBub2RlO1xuXG4gIGZvciAodmFyIGkgPSAwOyBpIDwgbE5vZGVzLmxlbmd0aDsgaSsrKVxuICB7XG4gICAgbm9kZSA9IGxOb2Rlc1tpXTtcbiAgICBub2RlLm1vdmUoKTtcbiAgfVxufVxuXG5GRExheW91dC5wcm90b3R5cGUuY2FsY1NwcmluZ0ZvcmNlID0gZnVuY3Rpb24gKGVkZ2UsIGlkZWFsTGVuZ3RoKSB7XG4gIHZhciBzb3VyY2VOb2RlID0gZWRnZS5nZXRTb3VyY2UoKTtcbiAgdmFyIHRhcmdldE5vZGUgPSBlZGdlLmdldFRhcmdldCgpO1xuXG4gIHZhciBsZW5ndGg7XG4gIHZhciBzcHJpbmdGb3JjZTtcbiAgdmFyIHNwcmluZ0ZvcmNlWDtcbiAgdmFyIHNwcmluZ0ZvcmNlWTtcblxuICAvLyBVcGRhdGUgZWRnZSBsZW5ndGhcbiAgaWYgKHRoaXMudW5pZm9ybUxlYWZOb2RlU2l6ZXMgJiZcbiAgICAgICAgICBzb3VyY2VOb2RlLmdldENoaWxkKCkgPT0gbnVsbCAmJiB0YXJnZXROb2RlLmdldENoaWxkKCkgPT0gbnVsbClcbiAge1xuICAgIGVkZ2UudXBkYXRlTGVuZ3RoU2ltcGxlKCk7XG4gIH1cbiAgZWxzZVxuICB7XG4gICAgZWRnZS51cGRhdGVMZW5ndGgoKTtcblxuICAgIGlmIChlZGdlLmlzT3ZlcmxhcGluZ1NvdXJjZUFuZFRhcmdldClcbiAgICB7XG4gICAgICByZXR1cm47XG4gICAgfVxuICB9XG5cbiAgbGVuZ3RoID0gZWRnZS5nZXRMZW5ndGgoKTtcblxuICAvLyBDYWxjdWxhdGUgc3ByaW5nIGZvcmNlc1xuICBzcHJpbmdGb3JjZSA9IHRoaXMuc3ByaW5nQ29uc3RhbnQgKiAobGVuZ3RoIC0gaWRlYWxMZW5ndGgpO1xuXG4gIC8vIFByb2plY3QgZm9yY2Ugb250byB4IGFuZCB5IGF4ZXNcbiAgc3ByaW5nRm9yY2VYID0gc3ByaW5nRm9yY2UgKiAoZWRnZS5sZW5ndGhYIC8gbGVuZ3RoKTtcbiAgc3ByaW5nRm9yY2VZID0gc3ByaW5nRm9yY2UgKiAoZWRnZS5sZW5ndGhZIC8gbGVuZ3RoKTtcblxuICAvLyBBcHBseSBmb3JjZXMgb24gdGhlIGVuZCBub2Rlc1xuICBzb3VyY2VOb2RlLnNwcmluZ0ZvcmNlWCArPSBzcHJpbmdGb3JjZVg7XG4gIHNvdXJjZU5vZGUuc3ByaW5nRm9yY2VZICs9IHNwcmluZ0ZvcmNlWTtcbiAgdGFyZ2V0Tm9kZS5zcHJpbmdGb3JjZVggLT0gc3ByaW5nRm9yY2VYO1xuICB0YXJnZXROb2RlLnNwcmluZ0ZvcmNlWSAtPSBzcHJpbmdGb3JjZVk7XG59O1xuXG5GRExheW91dC5wcm90b3R5cGUuY2FsY1JlcHVsc2lvbkZvcmNlID0gZnVuY3Rpb24gKG5vZGVBLCBub2RlQikge1xuICB2YXIgcmVjdEEgPSBub2RlQS5nZXRSZWN0KCk7XG4gIHZhciByZWN0QiA9IG5vZGVCLmdldFJlY3QoKTtcbiAgdmFyIG92ZXJsYXBBbW91bnQgPSBuZXcgQXJyYXkoMik7XG4gIHZhciBjbGlwUG9pbnRzID0gbmV3IEFycmF5KDQpO1xuICB2YXIgZGlzdGFuY2VYO1xuICB2YXIgZGlzdGFuY2VZO1xuICB2YXIgZGlzdGFuY2VTcXVhcmVkO1xuICB2YXIgZGlzdGFuY2U7XG4gIHZhciByZXB1bHNpb25Gb3JjZTtcbiAgdmFyIHJlcHVsc2lvbkZvcmNlWDtcbiAgdmFyIHJlcHVsc2lvbkZvcmNlWTtcblxuICBpZiAocmVjdEEuaW50ZXJzZWN0cyhyZWN0QikpLy8gdHdvIG5vZGVzIG92ZXJsYXBcbiAge1xuICAgIC8vIGNhbGN1bGF0ZSBzZXBhcmF0aW9uIGFtb3VudCBpbiB4IGFuZCB5IGRpcmVjdGlvbnNcbiAgICBJR2VvbWV0cnkuY2FsY1NlcGFyYXRpb25BbW91bnQocmVjdEEsXG4gICAgICAgICAgICByZWN0QixcbiAgICAgICAgICAgIG92ZXJsYXBBbW91bnQsXG4gICAgICAgICAgICBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIIC8gMi4wKTtcblxuICAgIHJlcHVsc2lvbkZvcmNlWCA9IG92ZXJsYXBBbW91bnRbMF07XG4gICAgcmVwdWxzaW9uRm9yY2VZID0gb3ZlcmxhcEFtb3VudFsxXTtcbiAgfVxuICBlbHNlLy8gbm8gb3ZlcmxhcFxuICB7XG4gICAgLy8gY2FsY3VsYXRlIGRpc3RhbmNlXG5cbiAgICBpZiAodGhpcy51bmlmb3JtTGVhZk5vZGVTaXplcyAmJlxuICAgICAgICAgICAgbm9kZUEuZ2V0Q2hpbGQoKSA9PSBudWxsICYmIG5vZGVCLmdldENoaWxkKCkgPT0gbnVsbCkvLyBzaW1wbHkgYmFzZSByZXB1bHNpb24gb24gZGlzdGFuY2Ugb2Ygbm9kZSBjZW50ZXJzXG4gICAge1xuICAgICAgZGlzdGFuY2VYID0gcmVjdEIuZ2V0Q2VudGVyWCgpIC0gcmVjdEEuZ2V0Q2VudGVyWCgpO1xuICAgICAgZGlzdGFuY2VZID0gcmVjdEIuZ2V0Q2VudGVyWSgpIC0gcmVjdEEuZ2V0Q2VudGVyWSgpO1xuICAgIH1cbiAgICBlbHNlLy8gdXNlIGNsaXBwaW5nIHBvaW50c1xuICAgIHtcbiAgICAgIElHZW9tZXRyeS5nZXRJbnRlcnNlY3Rpb24ocmVjdEEsIHJlY3RCLCBjbGlwUG9pbnRzKTtcblxuICAgICAgZGlzdGFuY2VYID0gY2xpcFBvaW50c1syXSAtIGNsaXBQb2ludHNbMF07XG4gICAgICBkaXN0YW5jZVkgPSBjbGlwUG9pbnRzWzNdIC0gY2xpcFBvaW50c1sxXTtcbiAgICB9XG5cbiAgICAvLyBObyByZXB1bHNpb24gcmFuZ2UuIEZSIGdyaWQgdmFyaWFudCBzaG91bGQgdGFrZSBjYXJlIG9mIHRoaXMuXG4gICAgaWYgKE1hdGguYWJzKGRpc3RhbmNlWCkgPCBGRExheW91dENvbnN0YW50cy5NSU5fUkVQVUxTSU9OX0RJU1QpXG4gICAge1xuICAgICAgZGlzdGFuY2VYID0gSU1hdGguc2lnbihkaXN0YW5jZVgpICpcbiAgICAgICAgICAgICAgRkRMYXlvdXRDb25zdGFudHMuTUlOX1JFUFVMU0lPTl9ESVNUO1xuICAgIH1cblxuICAgIGlmIChNYXRoLmFicyhkaXN0YW5jZVkpIDwgRkRMYXlvdXRDb25zdGFudHMuTUlOX1JFUFVMU0lPTl9ESVNUKVxuICAgIHtcbiAgICAgIGRpc3RhbmNlWSA9IElNYXRoLnNpZ24oZGlzdGFuY2VZKSAqXG4gICAgICAgICAgICAgIEZETGF5b3V0Q29uc3RhbnRzLk1JTl9SRVBVTFNJT05fRElTVDtcbiAgICB9XG5cbiAgICBkaXN0YW5jZVNxdWFyZWQgPSBkaXN0YW5jZVggKiBkaXN0YW5jZVggKyBkaXN0YW5jZVkgKiBkaXN0YW5jZVk7XG4gICAgZGlzdGFuY2UgPSBNYXRoLnNxcnQoZGlzdGFuY2VTcXVhcmVkKTtcblxuICAgIHJlcHVsc2lvbkZvcmNlID0gdGhpcy5yZXB1bHNpb25Db25zdGFudCAvIGRpc3RhbmNlU3F1YXJlZDtcblxuICAgIC8vIFByb2plY3QgZm9yY2Ugb250byB4IGFuZCB5IGF4ZXNcbiAgICByZXB1bHNpb25Gb3JjZVggPSByZXB1bHNpb25Gb3JjZSAqIGRpc3RhbmNlWCAvIGRpc3RhbmNlO1xuICAgIHJlcHVsc2lvbkZvcmNlWSA9IHJlcHVsc2lvbkZvcmNlICogZGlzdGFuY2VZIC8gZGlzdGFuY2U7XG4gIH1cblxuICAvLyBBcHBseSBmb3JjZXMgb24gdGhlIHR3byBub2Rlc1xuICBub2RlQS5yZXB1bHNpb25Gb3JjZVggLT0gcmVwdWxzaW9uRm9yY2VYO1xuICBub2RlQS5yZXB1bHNpb25Gb3JjZVkgLT0gcmVwdWxzaW9uRm9yY2VZO1xuICBub2RlQi5yZXB1bHNpb25Gb3JjZVggKz0gcmVwdWxzaW9uRm9yY2VYO1xuICBub2RlQi5yZXB1bHNpb25Gb3JjZVkgKz0gcmVwdWxzaW9uRm9yY2VZO1xufTtcblxuRkRMYXlvdXQucHJvdG90eXBlLmNhbGNHcmF2aXRhdGlvbmFsRm9yY2UgPSBmdW5jdGlvbiAobm9kZSkge1xuICB2YXIgb3duZXJHcmFwaDtcbiAgdmFyIG93bmVyQ2VudGVyWDtcbiAgdmFyIG93bmVyQ2VudGVyWTtcbiAgdmFyIGRpc3RhbmNlWDtcbiAgdmFyIGRpc3RhbmNlWTtcbiAgdmFyIGFic0Rpc3RhbmNlWDtcbiAgdmFyIGFic0Rpc3RhbmNlWTtcbiAgdmFyIGVzdGltYXRlZFNpemU7XG4gIG93bmVyR3JhcGggPSBub2RlLmdldE93bmVyKCk7XG5cbiAgb3duZXJDZW50ZXJYID0gKG93bmVyR3JhcGguZ2V0UmlnaHQoKSArIG93bmVyR3JhcGguZ2V0TGVmdCgpKSAvIDI7XG4gIG93bmVyQ2VudGVyWSA9IChvd25lckdyYXBoLmdldFRvcCgpICsgb3duZXJHcmFwaC5nZXRCb3R0b20oKSkgLyAyO1xuICBkaXN0YW5jZVggPSBub2RlLmdldENlbnRlclgoKSAtIG93bmVyQ2VudGVyWDtcbiAgZGlzdGFuY2VZID0gbm9kZS5nZXRDZW50ZXJZKCkgLSBvd25lckNlbnRlclk7XG4gIGFic0Rpc3RhbmNlWCA9IE1hdGguYWJzKGRpc3RhbmNlWCk7XG4gIGFic0Rpc3RhbmNlWSA9IE1hdGguYWJzKGRpc3RhbmNlWSk7XG5cbiAgaWYgKG5vZGUuZ2V0T3duZXIoKSA9PSB0aGlzLmdyYXBoTWFuYWdlci5nZXRSb290KCkpLy8gaW4gdGhlIHJvb3QgZ3JhcGhcbiAge1xuICAgIE1hdGguZmxvb3IoODApO1xuICAgIGVzdGltYXRlZFNpemUgPSBNYXRoLmZsb29yKG93bmVyR3JhcGguZ2V0RXN0aW1hdGVkU2l6ZSgpICpcbiAgICAgICAgICAgIHRoaXMuZ3Jhdml0eVJhbmdlRmFjdG9yKTtcblxuICAgIGlmIChhYnNEaXN0YW5jZVggPiBlc3RpbWF0ZWRTaXplIHx8IGFic0Rpc3RhbmNlWSA+IGVzdGltYXRlZFNpemUpXG4gICAge1xuICAgICAgbm9kZS5ncmF2aXRhdGlvbkZvcmNlWCA9IC10aGlzLmdyYXZpdHlDb25zdGFudCAqIGRpc3RhbmNlWDtcbiAgICAgIG5vZGUuZ3Jhdml0YXRpb25Gb3JjZVkgPSAtdGhpcy5ncmF2aXR5Q29uc3RhbnQgKiBkaXN0YW5jZVk7XG4gICAgfVxuICB9XG4gIGVsc2UvLyBpbnNpZGUgYSBjb21wb3VuZFxuICB7XG4gICAgZXN0aW1hdGVkU2l6ZSA9IE1hdGguZmxvb3IoKG93bmVyR3JhcGguZ2V0RXN0aW1hdGVkU2l6ZSgpICpcbiAgICAgICAgICAgIHRoaXMuY29tcG91bmRHcmF2aXR5UmFuZ2VGYWN0b3IpKTtcblxuICAgIGlmIChhYnNEaXN0YW5jZVggPiBlc3RpbWF0ZWRTaXplIHx8IGFic0Rpc3RhbmNlWSA+IGVzdGltYXRlZFNpemUpXG4gICAge1xuICAgICAgbm9kZS5ncmF2aXRhdGlvbkZvcmNlWCA9IC10aGlzLmdyYXZpdHlDb25zdGFudCAqIGRpc3RhbmNlWCAqXG4gICAgICAgICAgICAgIHRoaXMuY29tcG91bmRHcmF2aXR5Q29uc3RhbnQ7XG4gICAgICBub2RlLmdyYXZpdGF0aW9uRm9yY2VZID0gLXRoaXMuZ3Jhdml0eUNvbnN0YW50ICogZGlzdGFuY2VZICpcbiAgICAgICAgICAgICAgdGhpcy5jb21wb3VuZEdyYXZpdHlDb25zdGFudDtcbiAgICB9XG4gIH1cbn07XG5cbkZETGF5b3V0LnByb3RvdHlwZS5pc0NvbnZlcmdlZCA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIGNvbnZlcmdlZDtcbiAgdmFyIG9zY2lsYXRpbmcgPSBmYWxzZTtcblxuICBpZiAodGhpcy50b3RhbEl0ZXJhdGlvbnMgPiB0aGlzLm1heEl0ZXJhdGlvbnMgLyAzKVxuICB7XG4gICAgb3NjaWxhdGluZyA9XG4gICAgICAgICAgICBNYXRoLmFicyh0aGlzLnRvdGFsRGlzcGxhY2VtZW50IC0gdGhpcy5vbGRUb3RhbERpc3BsYWNlbWVudCkgPCAyO1xuICB9XG5cbiAgY29udmVyZ2VkID0gdGhpcy50b3RhbERpc3BsYWNlbWVudCA8IHRoaXMudG90YWxEaXNwbGFjZW1lbnRUaHJlc2hvbGQ7XG5cbiAgdGhpcy5vbGRUb3RhbERpc3BsYWNlbWVudCA9IHRoaXMudG90YWxEaXNwbGFjZW1lbnQ7XG5cbiAgcmV0dXJuIGNvbnZlcmdlZCB8fCBvc2NpbGF0aW5nO1xufTtcblxuRkRMYXlvdXQucHJvdG90eXBlLmFuaW1hdGUgPSBmdW5jdGlvbiAoKSB7XG4gIGlmICh0aGlzLmFuaW1hdGlvbkR1cmluZ0xheW91dCAmJiAhdGhpcy5pc1N1YkxheW91dClcbiAge1xuICAgIGlmICh0aGlzLm5vdEFuaW1hdGVkSXRlcmF0aW9ucyA9PSB0aGlzLmFuaW1hdGlvblBlcmlvZClcbiAgICB7XG4gICAgICB0aGlzLnVwZGF0ZSgpO1xuICAgICAgdGhpcy5ub3RBbmltYXRlZEl0ZXJhdGlvbnMgPSAwO1xuICAgIH1cbiAgICBlbHNlXG4gICAge1xuICAgICAgdGhpcy5ub3RBbmltYXRlZEl0ZXJhdGlvbnMrKztcbiAgICB9XG4gIH1cbn07XG5cbkZETGF5b3V0LnByb3RvdHlwZS5jYWxjUmVwdWxzaW9uUmFuZ2UgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiAwLjA7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IEZETGF5b3V0O1xuIiwidmFyIExheW91dENvbnN0YW50cyA9IHJlcXVpcmUoJy4vTGF5b3V0Q29uc3RhbnRzJyk7XG5cbmZ1bmN0aW9uIEZETGF5b3V0Q29uc3RhbnRzKCkge1xufVxuXG4vL0ZETGF5b3V0Q29uc3RhbnRzIGluaGVyaXRzIHN0YXRpYyBwcm9wcyBpbiBMYXlvdXRDb25zdGFudHNcbmZvciAodmFyIHByb3AgaW4gTGF5b3V0Q29uc3RhbnRzKSB7XG4gIEZETGF5b3V0Q29uc3RhbnRzW3Byb3BdID0gTGF5b3V0Q29uc3RhbnRzW3Byb3BdO1xufVxuXG5GRExheW91dENvbnN0YW50cy5NQVhfSVRFUkFUSU9OUyA9IDI1MDA7XG5cbkZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEggPSA1MDtcbkZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfU1BSSU5HX1NUUkVOR1RIID0gMC40NTtcbkZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfUkVQVUxTSU9OX1NUUkVOR1RIID0gNDUwMC4wO1xuRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1NUUkVOR1RIID0gMC40O1xuRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1NUUkVOR1RIID0gMS4wO1xuRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IDMuODtcbkZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9SQU5HRV9GQUNUT1IgPSAxLjU7XG5GRExheW91dENvbnN0YW50cy5ERUZBVUxUX1VTRV9TTUFSVF9JREVBTF9FREdFX0xFTkdUSF9DQUxDVUxBVElPTiA9IHRydWU7XG5GRExheW91dENvbnN0YW50cy5ERUZBVUxUX1VTRV9TTUFSVF9SRVBVTFNJT05fUkFOR0VfQ0FMQ1VMQVRJT04gPSB0cnVlO1xuRkRMYXlvdXRDb25zdGFudHMuTUFYX05PREVfRElTUExBQ0VNRU5UX0lOQ1JFTUVOVEFMID0gMTAwLjA7XG5GRExheW91dENvbnN0YW50cy5NQVhfTk9ERV9ESVNQTEFDRU1FTlQgPSBGRExheW91dENvbnN0YW50cy5NQVhfTk9ERV9ESVNQTEFDRU1FTlRfSU5DUkVNRU5UQUwgKiAzO1xuRkRMYXlvdXRDb25zdGFudHMuTUlOX1JFUFVMU0lPTl9ESVNUID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9FREdFX0xFTkdUSCAvIDEwLjA7XG5GRExheW91dENvbnN0YW50cy5DT05WRVJHRU5DRV9DSEVDS19QRVJJT0QgPSAxMDA7XG5GRExheW91dENvbnN0YW50cy5QRVJfTEVWRUxfSURFQUxfRURHRV9MRU5HVEhfRkFDVE9SID0gMC4xO1xuRkRMYXlvdXRDb25zdGFudHMuTUlOX0VER0VfTEVOR1RIID0gMTtcbkZETGF5b3V0Q29uc3RhbnRzLkdSSURfQ0FMQ1VMQVRJT05fQ0hFQ0tfUEVSSU9EID0gMTA7XG5cbm1vZHVsZS5leHBvcnRzID0gRkRMYXlvdXRDb25zdGFudHM7XG4iLCJ2YXIgTEVkZ2UgPSByZXF1aXJlKCcuL0xFZGdlJyk7XG52YXIgRkRMYXlvdXRDb25zdGFudHMgPSByZXF1aXJlKCcuL0ZETGF5b3V0Q29uc3RhbnRzJyk7XG5cbmZ1bmN0aW9uIEZETGF5b3V0RWRnZShzb3VyY2UsIHRhcmdldCwgdkVkZ2UpIHtcbiAgTEVkZ2UuY2FsbCh0aGlzLCBzb3VyY2UsIHRhcmdldCwgdkVkZ2UpO1xuICB0aGlzLmlkZWFsTGVuZ3RoID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9FREdFX0xFTkdUSDtcbn1cblxuRkRMYXlvdXRFZGdlLnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoTEVkZ2UucHJvdG90eXBlKTtcblxuZm9yICh2YXIgcHJvcCBpbiBMRWRnZSkge1xuICBGRExheW91dEVkZ2VbcHJvcF0gPSBMRWRnZVtwcm9wXTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBGRExheW91dEVkZ2U7XG4iLCJ2YXIgTE5vZGUgPSByZXF1aXJlKCcuL0xOb2RlJyk7XG5cbmZ1bmN0aW9uIEZETGF5b3V0Tm9kZShnbSwgbG9jLCBzaXplLCB2Tm9kZSkge1xuICAvLyBhbHRlcm5hdGl2ZSBjb25zdHJ1Y3RvciBpcyBoYW5kbGVkIGluc2lkZSBMTm9kZVxuICBMTm9kZS5jYWxsKHRoaXMsIGdtLCBsb2MsIHNpemUsIHZOb2RlKTtcbiAgLy9TcHJpbmcsIHJlcHVsc2lvbiBhbmQgZ3Jhdml0YXRpb25hbCBmb3JjZXMgYWN0aW5nIG9uIHRoaXMgbm9kZVxuICB0aGlzLnNwcmluZ0ZvcmNlWCA9IDA7XG4gIHRoaXMuc3ByaW5nRm9yY2VZID0gMDtcbiAgdGhpcy5yZXB1bHNpb25Gb3JjZVggPSAwO1xuICB0aGlzLnJlcHVsc2lvbkZvcmNlWSA9IDA7XG4gIHRoaXMuZ3Jhdml0YXRpb25Gb3JjZVggPSAwO1xuICB0aGlzLmdyYXZpdGF0aW9uRm9yY2VZID0gMDtcbiAgLy9BbW91bnQgYnkgd2hpY2ggdGhpcyBub2RlIGlzIHRvIGJlIG1vdmVkIGluIHRoaXMgaXRlcmF0aW9uXG4gIHRoaXMuZGlzcGxhY2VtZW50WCA9IDA7XG4gIHRoaXMuZGlzcGxhY2VtZW50WSA9IDA7XG5cbiAgLy9TdGFydCBhbmQgZmluaXNoIGdyaWQgY29vcmRpbmF0ZXMgdGhhdCB0aGlzIG5vZGUgaXMgZmFsbGVuIGludG9cbiAgdGhpcy5zdGFydFggPSAwO1xuICB0aGlzLmZpbmlzaFggPSAwO1xuICB0aGlzLnN0YXJ0WSA9IDA7XG4gIHRoaXMuZmluaXNoWSA9IDA7XG5cbiAgLy9HZW9tZXRyaWMgbmVpZ2hib3JzIG9mIHRoaXMgbm9kZVxuICB0aGlzLnN1cnJvdW5kaW5nID0gW107XG59XG5cbkZETGF5b3V0Tm9kZS5wcm90b3R5cGUgPSBPYmplY3QuY3JlYXRlKExOb2RlLnByb3RvdHlwZSk7XG5cbmZvciAodmFyIHByb3AgaW4gTE5vZGUpIHtcbiAgRkRMYXlvdXROb2RlW3Byb3BdID0gTE5vZGVbcHJvcF07XG59XG5cbkZETGF5b3V0Tm9kZS5wcm90b3R5cGUuc2V0R3JpZENvb3JkaW5hdGVzID0gZnVuY3Rpb24gKF9zdGFydFgsIF9maW5pc2hYLCBfc3RhcnRZLCBfZmluaXNoWSlcbntcbiAgdGhpcy5zdGFydFggPSBfc3RhcnRYO1xuICB0aGlzLmZpbmlzaFggPSBfZmluaXNoWDtcbiAgdGhpcy5zdGFydFkgPSBfc3RhcnRZO1xuICB0aGlzLmZpbmlzaFkgPSBfZmluaXNoWTtcblxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBGRExheW91dE5vZGU7XG4iLCJ2YXIgVW5pcXVlSURHZW5lcmV0b3IgPSByZXF1aXJlKCcuL1VuaXF1ZUlER2VuZXJldG9yJyk7XG5cbmZ1bmN0aW9uIEhhc2hNYXAoKSB7XG4gIHRoaXMubWFwID0ge307XG4gIHRoaXMua2V5cyA9IFtdO1xufVxuXG5IYXNoTWFwLnByb3RvdHlwZS5wdXQgPSBmdW5jdGlvbiAoa2V5LCB2YWx1ZSkge1xuICB2YXIgdGhlSWQgPSBVbmlxdWVJREdlbmVyZXRvci5jcmVhdGVJRChrZXkpO1xuICBpZiAoIXRoaXMuY29udGFpbnModGhlSWQpKSB7XG4gICAgdGhpcy5tYXBbdGhlSWRdID0gdmFsdWU7XG4gICAgdGhpcy5rZXlzLnB1c2goa2V5KTtcbiAgfVxufTtcblxuSGFzaE1hcC5wcm90b3R5cGUuY29udGFpbnMgPSBmdW5jdGlvbiAoa2V5KSB7XG4gIHZhciB0aGVJZCA9IFVuaXF1ZUlER2VuZXJldG9yLmNyZWF0ZUlEKGtleSk7XG4gIHJldHVybiB0aGlzLm1hcFtrZXldICE9IG51bGw7XG59O1xuXG5IYXNoTWFwLnByb3RvdHlwZS5nZXQgPSBmdW5jdGlvbiAoa2V5KSB7XG4gIHZhciB0aGVJZCA9IFVuaXF1ZUlER2VuZXJldG9yLmNyZWF0ZUlEKGtleSk7XG4gIHJldHVybiB0aGlzLm1hcFt0aGVJZF07XG59O1xuXG5IYXNoTWFwLnByb3RvdHlwZS5rZXlTZXQgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiB0aGlzLmtleXM7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IEhhc2hNYXA7XG4iLCJ2YXIgVW5pcXVlSURHZW5lcmV0b3IgPSByZXF1aXJlKCcuL1VuaXF1ZUlER2VuZXJldG9yJyk7XG5cbmZ1bmN0aW9uIEhhc2hTZXQoKSB7XG4gIHRoaXMuc2V0ID0ge307XG59XG47XG5cbkhhc2hTZXQucHJvdG90eXBlLmFkZCA9IGZ1bmN0aW9uIChvYmopIHtcbiAgdmFyIHRoZUlkID0gVW5pcXVlSURHZW5lcmV0b3IuY3JlYXRlSUQob2JqKTtcbiAgaWYgKCF0aGlzLmNvbnRhaW5zKHRoZUlkKSlcbiAgICB0aGlzLnNldFt0aGVJZF0gPSBvYmo7XG59O1xuXG5IYXNoU2V0LnByb3RvdHlwZS5yZW1vdmUgPSBmdW5jdGlvbiAob2JqKSB7XG4gIGRlbGV0ZSB0aGlzLnNldFtVbmlxdWVJREdlbmVyZXRvci5jcmVhdGVJRChvYmopXTtcbn07XG5cbkhhc2hTZXQucHJvdG90eXBlLmNsZWFyID0gZnVuY3Rpb24gKCkge1xuICB0aGlzLnNldCA9IHt9O1xufTtcblxuSGFzaFNldC5wcm90b3R5cGUuY29udGFpbnMgPSBmdW5jdGlvbiAob2JqKSB7XG4gIHJldHVybiB0aGlzLnNldFtVbmlxdWVJREdlbmVyZXRvci5jcmVhdGVJRChvYmopXSA9PSBvYmo7XG59O1xuXG5IYXNoU2V0LnByb3RvdHlwZS5pc0VtcHR5ID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gdGhpcy5zaXplKCkgPT09IDA7XG59O1xuXG5IYXNoU2V0LnByb3RvdHlwZS5zaXplID0gZnVuY3Rpb24gKCkge1xuICByZXR1cm4gT2JqZWN0LmtleXModGhpcy5zZXQpLmxlbmd0aDtcbn07XG5cbi8vY29uY2F0cyB0aGlzLnNldCB0byB0aGUgZ2l2ZW4gbGlzdFxuSGFzaFNldC5wcm90b3R5cGUuYWRkQWxsVG8gPSBmdW5jdGlvbiAobGlzdCkge1xuICB2YXIga2V5cyA9IE9iamVjdC5rZXlzKHRoaXMuc2V0KTtcbiAgdmFyIGxlbmd0aCA9IGtleXMubGVuZ3RoO1xuICBmb3IgKHZhciBpID0gMDsgaSA8IGxlbmd0aDsgaSsrKSB7XG4gICAgbGlzdC5wdXNoKHRoaXMuc2V0W2tleXNbaV1dKTtcbiAgfVxufTtcblxuSGFzaFNldC5wcm90b3R5cGUuc2l6ZSA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIE9iamVjdC5rZXlzKHRoaXMuc2V0KS5sZW5ndGg7XG59O1xuXG5IYXNoU2V0LnByb3RvdHlwZS5hZGRBbGwgPSBmdW5jdGlvbiAobGlzdCkge1xuICB2YXIgcyA9IGxpc3QubGVuZ3RoO1xuICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKykge1xuICAgIHZhciB2ID0gbGlzdFtpXTtcbiAgICB0aGlzLmFkZCh2KTtcbiAgfVxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBIYXNoU2V0O1xuIiwiZnVuY3Rpb24gSUdlb21ldHJ5KCkge1xufVxuXG5JR2VvbWV0cnkuY2FsY1NlcGFyYXRpb25BbW91bnQgPSBmdW5jdGlvbiAocmVjdEEsIHJlY3RCLCBvdmVybGFwQW1vdW50LCBzZXBhcmF0aW9uQnVmZmVyKVxue1xuICBpZiAoIXJlY3RBLmludGVyc2VjdHMocmVjdEIpKSB7XG4gICAgdGhyb3cgXCJhc3NlcnQgZmFpbGVkXCI7XG4gIH1cbiAgdmFyIGRpcmVjdGlvbnMgPSBuZXcgQXJyYXkoMik7XG4gIElHZW9tZXRyeS5kZWNpZGVEaXJlY3Rpb25zRm9yT3ZlcmxhcHBpbmdOb2RlcyhyZWN0QSwgcmVjdEIsIGRpcmVjdGlvbnMpO1xuICBvdmVybGFwQW1vdW50WzBdID0gTWF0aC5taW4ocmVjdEEuZ2V0UmlnaHQoKSwgcmVjdEIuZ2V0UmlnaHQoKSkgLVxuICAgICAgICAgIE1hdGgubWF4KHJlY3RBLngsIHJlY3RCLngpO1xuICBvdmVybGFwQW1vdW50WzFdID0gTWF0aC5taW4ocmVjdEEuZ2V0Qm90dG9tKCksIHJlY3RCLmdldEJvdHRvbSgpKSAtXG4gICAgICAgICAgTWF0aC5tYXgocmVjdEEueSwgcmVjdEIueSk7XG4gIC8vIHVwZGF0ZSB0aGUgb3ZlcmxhcHBpbmcgYW1vdW50cyBmb3IgdGhlIGZvbGxvd2luZyBjYXNlczpcbiAgaWYgKChyZWN0QS5nZXRYKCkgPD0gcmVjdEIuZ2V0WCgpKSAmJiAocmVjdEEuZ2V0UmlnaHQoKSA+PSByZWN0Qi5nZXRSaWdodCgpKSlcbiAge1xuICAgIG92ZXJsYXBBbW91bnRbMF0gKz0gTWF0aC5taW4oKHJlY3RCLmdldFgoKSAtIHJlY3RBLmdldFgoKSksXG4gICAgICAgICAgICAocmVjdEEuZ2V0UmlnaHQoKSAtIHJlY3RCLmdldFJpZ2h0KCkpKTtcbiAgfVxuICBlbHNlIGlmICgocmVjdEIuZ2V0WCgpIDw9IHJlY3RBLmdldFgoKSkgJiYgKHJlY3RCLmdldFJpZ2h0KCkgPj0gcmVjdEEuZ2V0UmlnaHQoKSkpXG4gIHtcbiAgICBvdmVybGFwQW1vdW50WzBdICs9IE1hdGgubWluKChyZWN0QS5nZXRYKCkgLSByZWN0Qi5nZXRYKCkpLFxuICAgICAgICAgICAgKHJlY3RCLmdldFJpZ2h0KCkgLSByZWN0QS5nZXRSaWdodCgpKSk7XG4gIH1cbiAgaWYgKChyZWN0QS5nZXRZKCkgPD0gcmVjdEIuZ2V0WSgpKSAmJiAocmVjdEEuZ2V0Qm90dG9tKCkgPj0gcmVjdEIuZ2V0Qm90dG9tKCkpKVxuICB7XG4gICAgb3ZlcmxhcEFtb3VudFsxXSArPSBNYXRoLm1pbigocmVjdEIuZ2V0WSgpIC0gcmVjdEEuZ2V0WSgpKSxcbiAgICAgICAgICAgIChyZWN0QS5nZXRCb3R0b20oKSAtIHJlY3RCLmdldEJvdHRvbSgpKSk7XG4gIH1cbiAgZWxzZSBpZiAoKHJlY3RCLmdldFkoKSA8PSByZWN0QS5nZXRZKCkpICYmIChyZWN0Qi5nZXRCb3R0b20oKSA+PSByZWN0QS5nZXRCb3R0b20oKSkpXG4gIHtcbiAgICBvdmVybGFwQW1vdW50WzFdICs9IE1hdGgubWluKChyZWN0QS5nZXRZKCkgLSByZWN0Qi5nZXRZKCkpLFxuICAgICAgICAgICAgKHJlY3RCLmdldEJvdHRvbSgpIC0gcmVjdEEuZ2V0Qm90dG9tKCkpKTtcbiAgfVxuXG4gIC8vIGZpbmQgc2xvcGUgb2YgdGhlIGxpbmUgcGFzc2VzIHR3byBjZW50ZXJzXG4gIHZhciBzbG9wZSA9IE1hdGguYWJzKChyZWN0Qi5nZXRDZW50ZXJZKCkgLSByZWN0QS5nZXRDZW50ZXJZKCkpIC9cbiAgICAgICAgICAocmVjdEIuZ2V0Q2VudGVyWCgpIC0gcmVjdEEuZ2V0Q2VudGVyWCgpKSk7XG4gIC8vIGlmIGNlbnRlcnMgYXJlIG92ZXJsYXBwZWRcbiAgaWYgKChyZWN0Qi5nZXRDZW50ZXJZKCkgPT0gcmVjdEEuZ2V0Q2VudGVyWSgpKSAmJlxuICAgICAgICAgIChyZWN0Qi5nZXRDZW50ZXJYKCkgPT0gcmVjdEEuZ2V0Q2VudGVyWCgpKSlcbiAge1xuICAgIC8vIGFzc3VtZSB0aGUgc2xvcGUgaXMgMSAoNDUgZGVncmVlKVxuICAgIHNsb3BlID0gMS4wO1xuICB9XG5cbiAgdmFyIG1vdmVCeVkgPSBzbG9wZSAqIG92ZXJsYXBBbW91bnRbMF07XG4gIHZhciBtb3ZlQnlYID0gb3ZlcmxhcEFtb3VudFsxXSAvIHNsb3BlO1xuICBpZiAob3ZlcmxhcEFtb3VudFswXSA8IG1vdmVCeVgpXG4gIHtcbiAgICBtb3ZlQnlYID0gb3ZlcmxhcEFtb3VudFswXTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICBtb3ZlQnlZID0gb3ZlcmxhcEFtb3VudFsxXTtcbiAgfVxuICAvLyByZXR1cm4gaGFsZiB0aGUgYW1vdW50IHNvIHRoYXQgaWYgZWFjaCByZWN0YW5nbGUgaXMgbW92ZWQgYnkgdGhlc2VcbiAgLy8gYW1vdW50cyBpbiBvcHBvc2l0ZSBkaXJlY3Rpb25zLCBvdmVybGFwIHdpbGwgYmUgcmVzb2x2ZWRcbiAgb3ZlcmxhcEFtb3VudFswXSA9IC0xICogZGlyZWN0aW9uc1swXSAqICgobW92ZUJ5WCAvIDIpICsgc2VwYXJhdGlvbkJ1ZmZlcik7XG4gIG92ZXJsYXBBbW91bnRbMV0gPSAtMSAqIGRpcmVjdGlvbnNbMV0gKiAoKG1vdmVCeVkgLyAyKSArIHNlcGFyYXRpb25CdWZmZXIpO1xufVxuXG5JR2VvbWV0cnkuZGVjaWRlRGlyZWN0aW9uc0Zvck92ZXJsYXBwaW5nTm9kZXMgPSBmdW5jdGlvbiAocmVjdEEsIHJlY3RCLCBkaXJlY3Rpb25zKVxue1xuICBpZiAocmVjdEEuZ2V0Q2VudGVyWCgpIDwgcmVjdEIuZ2V0Q2VudGVyWCgpKVxuICB7XG4gICAgZGlyZWN0aW9uc1swXSA9IC0xO1xuICB9XG4gIGVsc2VcbiAge1xuICAgIGRpcmVjdGlvbnNbMF0gPSAxO1xuICB9XG5cbiAgaWYgKHJlY3RBLmdldENlbnRlclkoKSA8IHJlY3RCLmdldENlbnRlclkoKSlcbiAge1xuICAgIGRpcmVjdGlvbnNbMV0gPSAtMTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICBkaXJlY3Rpb25zWzFdID0gMTtcbiAgfVxufVxuXG5JR2VvbWV0cnkuZ2V0SW50ZXJzZWN0aW9uMiA9IGZ1bmN0aW9uIChyZWN0QSwgcmVjdEIsIHJlc3VsdClcbntcbiAgLy9yZXN1bHRbMC0xXSB3aWxsIGNvbnRhaW4gY2xpcFBvaW50IG9mIHJlY3RBLCByZXN1bHRbMi0zXSB3aWxsIGNvbnRhaW4gY2xpcFBvaW50IG9mIHJlY3RCXG4gIHZhciBwMXggPSByZWN0QS5nZXRDZW50ZXJYKCk7XG4gIHZhciBwMXkgPSByZWN0QS5nZXRDZW50ZXJZKCk7XG4gIHZhciBwMnggPSByZWN0Qi5nZXRDZW50ZXJYKCk7XG4gIHZhciBwMnkgPSByZWN0Qi5nZXRDZW50ZXJZKCk7XG5cbiAgLy9pZiB0d28gcmVjdGFuZ2xlcyBpbnRlcnNlY3QsIHRoZW4gY2xpcHBpbmcgcG9pbnRzIGFyZSBjZW50ZXJzXG4gIGlmIChyZWN0QS5pbnRlcnNlY3RzKHJlY3RCKSlcbiAge1xuICAgIHJlc3VsdFswXSA9IHAxeDtcbiAgICByZXN1bHRbMV0gPSBwMXk7XG4gICAgcmVzdWx0WzJdID0gcDJ4O1xuICAgIHJlc3VsdFszXSA9IHAyeTtcbiAgICByZXR1cm4gdHJ1ZTtcbiAgfVxuICAvL3ZhcmlhYmxlcyBmb3IgcmVjdEFcbiAgdmFyIHRvcExlZnRBeCA9IHJlY3RBLmdldFgoKTtcbiAgdmFyIHRvcExlZnRBeSA9IHJlY3RBLmdldFkoKTtcbiAgdmFyIHRvcFJpZ2h0QXggPSByZWN0QS5nZXRSaWdodCgpO1xuICB2YXIgYm90dG9tTGVmdEF4ID0gcmVjdEEuZ2V0WCgpO1xuICB2YXIgYm90dG9tTGVmdEF5ID0gcmVjdEEuZ2V0Qm90dG9tKCk7XG4gIHZhciBib3R0b21SaWdodEF4ID0gcmVjdEEuZ2V0UmlnaHQoKTtcbiAgdmFyIGhhbGZXaWR0aEEgPSByZWN0QS5nZXRXaWR0aEhhbGYoKTtcbiAgdmFyIGhhbGZIZWlnaHRBID0gcmVjdEEuZ2V0SGVpZ2h0SGFsZigpO1xuICAvL3ZhcmlhYmxlcyBmb3IgcmVjdEJcbiAgdmFyIHRvcExlZnRCeCA9IHJlY3RCLmdldFgoKTtcbiAgdmFyIHRvcExlZnRCeSA9IHJlY3RCLmdldFkoKTtcbiAgdmFyIHRvcFJpZ2h0QnggPSByZWN0Qi5nZXRSaWdodCgpO1xuICB2YXIgYm90dG9tTGVmdEJ4ID0gcmVjdEIuZ2V0WCgpO1xuICB2YXIgYm90dG9tTGVmdEJ5ID0gcmVjdEIuZ2V0Qm90dG9tKCk7XG4gIHZhciBib3R0b21SaWdodEJ4ID0gcmVjdEIuZ2V0UmlnaHQoKTtcbiAgdmFyIGhhbGZXaWR0aEIgPSByZWN0Qi5nZXRXaWR0aEhhbGYoKTtcbiAgdmFyIGhhbGZIZWlnaHRCID0gcmVjdEIuZ2V0SGVpZ2h0SGFsZigpO1xuICAvL2ZsYWcgd2hldGhlciBjbGlwcGluZyBwb2ludHMgYXJlIGZvdW5kXG4gIHZhciBjbGlwUG9pbnRBRm91bmQgPSBmYWxzZTtcbiAgdmFyIGNsaXBQb2ludEJGb3VuZCA9IGZhbHNlO1xuXG4gIC8vIGxpbmUgaXMgdmVydGljYWxcbiAgaWYgKHAxeCA9PSBwMngpXG4gIHtcbiAgICBpZiAocDF5ID4gcDJ5KVxuICAgIHtcbiAgICAgIHJlc3VsdFswXSA9IHAxeDtcbiAgICAgIHJlc3VsdFsxXSA9IHRvcExlZnRBeTtcbiAgICAgIHJlc3VsdFsyXSA9IHAyeDtcbiAgICAgIHJlc3VsdFszXSA9IGJvdHRvbUxlZnRCeTtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG4gICAgZWxzZSBpZiAocDF5IDwgcDJ5KVxuICAgIHtcbiAgICAgIHJlc3VsdFswXSA9IHAxeDtcbiAgICAgIHJlc3VsdFsxXSA9IGJvdHRvbUxlZnRBeTtcbiAgICAgIHJlc3VsdFsyXSA9IHAyeDtcbiAgICAgIHJlc3VsdFszXSA9IHRvcExlZnRCeTtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG4gICAgZWxzZVxuICAgIHtcbiAgICAgIC8vbm90IGxpbmUsIHJldHVybiBudWxsO1xuICAgIH1cbiAgfVxuICAvLyBsaW5lIGlzIGhvcml6b250YWxcbiAgZWxzZSBpZiAocDF5ID09IHAyeSlcbiAge1xuICAgIGlmIChwMXggPiBwMngpXG4gICAge1xuICAgICAgcmVzdWx0WzBdID0gdG9wTGVmdEF4O1xuICAgICAgcmVzdWx0WzFdID0gcDF5O1xuICAgICAgcmVzdWx0WzJdID0gdG9wUmlnaHRCeDtcbiAgICAgIHJlc3VsdFszXSA9IHAyeTtcbiAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG4gICAgZWxzZSBpZiAocDF4IDwgcDJ4KVxuICAgIHtcbiAgICAgIHJlc3VsdFswXSA9IHRvcFJpZ2h0QXg7XG4gICAgICByZXN1bHRbMV0gPSBwMXk7XG4gICAgICByZXN1bHRbMl0gPSB0b3BMZWZ0Qng7XG4gICAgICByZXN1bHRbM10gPSBwMnk7XG4gICAgICByZXR1cm4gZmFsc2U7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICAvL25vdCB2YWxpZCBsaW5lLCByZXR1cm4gbnVsbDtcbiAgICB9XG4gIH1cbiAgZWxzZVxuICB7XG4gICAgLy9zbG9wZXMgb2YgcmVjdEEncyBhbmQgcmVjdEIncyBkaWFnb25hbHNcbiAgICB2YXIgc2xvcGVBID0gcmVjdEEuaGVpZ2h0IC8gcmVjdEEud2lkdGg7XG4gICAgdmFyIHNsb3BlQiA9IHJlY3RCLmhlaWdodCAvIHJlY3RCLndpZHRoO1xuXG4gICAgLy9zbG9wZSBvZiBsaW5lIGJldHdlZW4gY2VudGVyIG9mIHJlY3RBIGFuZCBjZW50ZXIgb2YgcmVjdEJcbiAgICB2YXIgc2xvcGVQcmltZSA9IChwMnkgLSBwMXkpIC8gKHAyeCAtIHAxeCk7XG4gICAgdmFyIGNhcmRpbmFsRGlyZWN0aW9uQTtcbiAgICB2YXIgY2FyZGluYWxEaXJlY3Rpb25CO1xuICAgIHZhciB0ZW1wUG9pbnRBeDtcbiAgICB2YXIgdGVtcFBvaW50QXk7XG4gICAgdmFyIHRlbXBQb2ludEJ4O1xuICAgIHZhciB0ZW1wUG9pbnRCeTtcblxuICAgIC8vZGV0ZXJtaW5lIHdoZXRoZXIgY2xpcHBpbmcgcG9pbnQgaXMgdGhlIGNvcm5lciBvZiBub2RlQVxuICAgIGlmICgoLXNsb3BlQSkgPT0gc2xvcGVQcmltZSlcbiAgICB7XG4gICAgICBpZiAocDF4ID4gcDJ4KVxuICAgICAge1xuICAgICAgICByZXN1bHRbMF0gPSBib3R0b21MZWZ0QXg7XG4gICAgICAgIHJlc3VsdFsxXSA9IGJvdHRvbUxlZnRBeTtcbiAgICAgICAgY2xpcFBvaW50QUZvdW5kID0gdHJ1ZTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgcmVzdWx0WzBdID0gdG9wUmlnaHRBeDtcbiAgICAgICAgcmVzdWx0WzFdID0gdG9wTGVmdEF5O1xuICAgICAgICBjbGlwUG9pbnRBRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgIH1cbiAgICBlbHNlIGlmIChzbG9wZUEgPT0gc2xvcGVQcmltZSlcbiAgICB7XG4gICAgICBpZiAocDF4ID4gcDJ4KVxuICAgICAge1xuICAgICAgICByZXN1bHRbMF0gPSB0b3BMZWZ0QXg7XG4gICAgICAgIHJlc3VsdFsxXSA9IHRvcExlZnRBeTtcbiAgICAgICAgY2xpcFBvaW50QUZvdW5kID0gdHJ1ZTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgcmVzdWx0WzBdID0gYm90dG9tUmlnaHRBeDtcbiAgICAgICAgcmVzdWx0WzFdID0gYm90dG9tTGVmdEF5O1xuICAgICAgICBjbGlwUG9pbnRBRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgIH1cblxuICAgIC8vZGV0ZXJtaW5lIHdoZXRoZXIgY2xpcHBpbmcgcG9pbnQgaXMgdGhlIGNvcm5lciBvZiBub2RlQlxuICAgIGlmICgoLXNsb3BlQikgPT0gc2xvcGVQcmltZSlcbiAgICB7XG4gICAgICBpZiAocDJ4ID4gcDF4KVxuICAgICAge1xuICAgICAgICByZXN1bHRbMl0gPSBib3R0b21MZWZ0Qng7XG4gICAgICAgIHJlc3VsdFszXSA9IGJvdHRvbUxlZnRCeTtcbiAgICAgICAgY2xpcFBvaW50QkZvdW5kID0gdHJ1ZTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgcmVzdWx0WzJdID0gdG9wUmlnaHRCeDtcbiAgICAgICAgcmVzdWx0WzNdID0gdG9wTGVmdEJ5O1xuICAgICAgICBjbGlwUG9pbnRCRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgIH1cbiAgICBlbHNlIGlmIChzbG9wZUIgPT0gc2xvcGVQcmltZSlcbiAgICB7XG4gICAgICBpZiAocDJ4ID4gcDF4KVxuICAgICAge1xuICAgICAgICByZXN1bHRbMl0gPSB0b3BMZWZ0Qng7XG4gICAgICAgIHJlc3VsdFszXSA9IHRvcExlZnRCeTtcbiAgICAgICAgY2xpcFBvaW50QkZvdW5kID0gdHJ1ZTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgcmVzdWx0WzJdID0gYm90dG9tUmlnaHRCeDtcbiAgICAgICAgcmVzdWx0WzNdID0gYm90dG9tTGVmdEJ5O1xuICAgICAgICBjbGlwUG9pbnRCRm91bmQgPSB0cnVlO1xuICAgICAgfVxuICAgIH1cblxuICAgIC8vaWYgYm90aCBjbGlwcGluZyBwb2ludHMgYXJlIGNvcm5lcnNcbiAgICBpZiAoY2xpcFBvaW50QUZvdW5kICYmIGNsaXBQb2ludEJGb3VuZClcbiAgICB7XG4gICAgICByZXR1cm4gZmFsc2U7XG4gICAgfVxuXG4gICAgLy9kZXRlcm1pbmUgQ2FyZGluYWwgRGlyZWN0aW9uIG9mIHJlY3RhbmdsZXNcbiAgICBpZiAocDF4ID4gcDJ4KVxuICAgIHtcbiAgICAgIGlmIChwMXkgPiBwMnkpXG4gICAgICB7XG4gICAgICAgIGNhcmRpbmFsRGlyZWN0aW9uQSA9IElHZW9tZXRyeS5nZXRDYXJkaW5hbERpcmVjdGlvbihzbG9wZUEsIHNsb3BlUHJpbWUsIDQpO1xuICAgICAgICBjYXJkaW5hbERpcmVjdGlvbkIgPSBJR2VvbWV0cnkuZ2V0Q2FyZGluYWxEaXJlY3Rpb24oc2xvcGVCLCBzbG9wZVByaW1lLCAyKTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgY2FyZGluYWxEaXJlY3Rpb25BID0gSUdlb21ldHJ5LmdldENhcmRpbmFsRGlyZWN0aW9uKC1zbG9wZUEsIHNsb3BlUHJpbWUsIDMpO1xuICAgICAgICBjYXJkaW5hbERpcmVjdGlvbkIgPSBJR2VvbWV0cnkuZ2V0Q2FyZGluYWxEaXJlY3Rpb24oLXNsb3BlQiwgc2xvcGVQcmltZSwgMSk7XG4gICAgICB9XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICBpZiAocDF5ID4gcDJ5KVxuICAgICAge1xuICAgICAgICBjYXJkaW5hbERpcmVjdGlvbkEgPSBJR2VvbWV0cnkuZ2V0Q2FyZGluYWxEaXJlY3Rpb24oLXNsb3BlQSwgc2xvcGVQcmltZSwgMSk7XG4gICAgICAgIGNhcmRpbmFsRGlyZWN0aW9uQiA9IElHZW9tZXRyeS5nZXRDYXJkaW5hbERpcmVjdGlvbigtc2xvcGVCLCBzbG9wZVByaW1lLCAzKTtcbiAgICAgIH1cbiAgICAgIGVsc2VcbiAgICAgIHtcbiAgICAgICAgY2FyZGluYWxEaXJlY3Rpb25BID0gSUdlb21ldHJ5LmdldENhcmRpbmFsRGlyZWN0aW9uKHNsb3BlQSwgc2xvcGVQcmltZSwgMik7XG4gICAgICAgIGNhcmRpbmFsRGlyZWN0aW9uQiA9IElHZW9tZXRyeS5nZXRDYXJkaW5hbERpcmVjdGlvbihzbG9wZUIsIHNsb3BlUHJpbWUsIDQpO1xuICAgICAgfVxuICAgIH1cbiAgICAvL2NhbGN1bGF0ZSBjbGlwcGluZyBQb2ludCBpZiBpdCBpcyBub3QgZm91bmQgYmVmb3JlXG4gICAgaWYgKCFjbGlwUG9pbnRBRm91bmQpXG4gICAge1xuICAgICAgc3dpdGNoIChjYXJkaW5hbERpcmVjdGlvbkEpXG4gICAgICB7XG4gICAgICAgIGNhc2UgMTpcbiAgICAgICAgICB0ZW1wUG9pbnRBeSA9IHRvcExlZnRBeTtcbiAgICAgICAgICB0ZW1wUG9pbnRBeCA9IHAxeCArICgtaGFsZkhlaWdodEEpIC8gc2xvcGVQcmltZTtcbiAgICAgICAgICByZXN1bHRbMF0gPSB0ZW1wUG9pbnRBeDtcbiAgICAgICAgICByZXN1bHRbMV0gPSB0ZW1wUG9pbnRBeTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgY2FzZSAyOlxuICAgICAgICAgIHRlbXBQb2ludEF4ID0gYm90dG9tUmlnaHRBeDtcbiAgICAgICAgICB0ZW1wUG9pbnRBeSA9IHAxeSArIGhhbGZXaWR0aEEgKiBzbG9wZVByaW1lO1xuICAgICAgICAgIHJlc3VsdFswXSA9IHRlbXBQb2ludEF4O1xuICAgICAgICAgIHJlc3VsdFsxXSA9IHRlbXBQb2ludEF5O1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICBjYXNlIDM6XG4gICAgICAgICAgdGVtcFBvaW50QXkgPSBib3R0b21MZWZ0QXk7XG4gICAgICAgICAgdGVtcFBvaW50QXggPSBwMXggKyBoYWxmSGVpZ2h0QSAvIHNsb3BlUHJpbWU7XG4gICAgICAgICAgcmVzdWx0WzBdID0gdGVtcFBvaW50QXg7XG4gICAgICAgICAgcmVzdWx0WzFdID0gdGVtcFBvaW50QXk7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIGNhc2UgNDpcbiAgICAgICAgICB0ZW1wUG9pbnRBeCA9IGJvdHRvbUxlZnRBeDtcbiAgICAgICAgICB0ZW1wUG9pbnRBeSA9IHAxeSArICgtaGFsZldpZHRoQSkgKiBzbG9wZVByaW1lO1xuICAgICAgICAgIHJlc3VsdFswXSA9IHRlbXBQb2ludEF4O1xuICAgICAgICAgIHJlc3VsdFsxXSA9IHRlbXBQb2ludEF5O1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgfVxuICAgIH1cbiAgICBpZiAoIWNsaXBQb2ludEJGb3VuZClcbiAgICB7XG4gICAgICBzd2l0Y2ggKGNhcmRpbmFsRGlyZWN0aW9uQilcbiAgICAgIHtcbiAgICAgICAgY2FzZSAxOlxuICAgICAgICAgIHRlbXBQb2ludEJ5ID0gdG9wTGVmdEJ5O1xuICAgICAgICAgIHRlbXBQb2ludEJ4ID0gcDJ4ICsgKC1oYWxmSGVpZ2h0QikgLyBzbG9wZVByaW1lO1xuICAgICAgICAgIHJlc3VsdFsyXSA9IHRlbXBQb2ludEJ4O1xuICAgICAgICAgIHJlc3VsdFszXSA9IHRlbXBQb2ludEJ5O1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICBjYXNlIDI6XG4gICAgICAgICAgdGVtcFBvaW50QnggPSBib3R0b21SaWdodEJ4O1xuICAgICAgICAgIHRlbXBQb2ludEJ5ID0gcDJ5ICsgaGFsZldpZHRoQiAqIHNsb3BlUHJpbWU7XG4gICAgICAgICAgcmVzdWx0WzJdID0gdGVtcFBvaW50Qng7XG4gICAgICAgICAgcmVzdWx0WzNdID0gdGVtcFBvaW50Qnk7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICAgIGNhc2UgMzpcbiAgICAgICAgICB0ZW1wUG9pbnRCeSA9IGJvdHRvbUxlZnRCeTtcbiAgICAgICAgICB0ZW1wUG9pbnRCeCA9IHAyeCArIGhhbGZIZWlnaHRCIC8gc2xvcGVQcmltZTtcbiAgICAgICAgICByZXN1bHRbMl0gPSB0ZW1wUG9pbnRCeDtcbiAgICAgICAgICByZXN1bHRbM10gPSB0ZW1wUG9pbnRCeTtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgY2FzZSA0OlxuICAgICAgICAgIHRlbXBQb2ludEJ4ID0gYm90dG9tTGVmdEJ4O1xuICAgICAgICAgIHRlbXBQb2ludEJ5ID0gcDJ5ICsgKC1oYWxmV2lkdGhCKSAqIHNsb3BlUHJpbWU7XG4gICAgICAgICAgcmVzdWx0WzJdID0gdGVtcFBvaW50Qng7XG4gICAgICAgICAgcmVzdWx0WzNdID0gdGVtcFBvaW50Qnk7XG4gICAgICAgICAgYnJlYWs7XG4gICAgICB9XG4gICAgfVxuICB9XG4gIHJldHVybiBmYWxzZTtcbn1cblxuSUdlb21ldHJ5LmdldENhcmRpbmFsRGlyZWN0aW9uID0gZnVuY3Rpb24gKHNsb3BlLCBzbG9wZVByaW1lLCBsaW5lKVxue1xuICBpZiAoc2xvcGUgPiBzbG9wZVByaW1lKVxuICB7XG4gICAgcmV0dXJuIGxpbmU7XG4gIH1cbiAgZWxzZVxuICB7XG4gICAgcmV0dXJuIDEgKyBsaW5lICUgNDtcbiAgfVxufVxuXG5JR2VvbWV0cnkuZ2V0SW50ZXJzZWN0aW9uID0gZnVuY3Rpb24gKHMxLCBzMiwgZjEsIGYyKVxue1xuICBpZiAoZjIgPT0gbnVsbCkge1xuICAgIHJldHVybiBJR2VvbWV0cnkuZ2V0SW50ZXJzZWN0aW9uMihzMSwgczIsIGYxKTtcbiAgfVxuICB2YXIgeDEgPSBzMS54O1xuICB2YXIgeTEgPSBzMS55O1xuICB2YXIgeDIgPSBzMi54O1xuICB2YXIgeTIgPSBzMi55O1xuICB2YXIgeDMgPSBmMS54O1xuICB2YXIgeTMgPSBmMS55O1xuICB2YXIgeDQgPSBmMi54O1xuICB2YXIgeTQgPSBmMi55O1xuICB2YXIgeCwgeTsgLy8gaW50ZXJzZWN0aW9uIHBvaW50XG4gIHZhciBhMSwgYTIsIGIxLCBiMiwgYzEsIGMyOyAvLyBjb2VmZmljaWVudHMgb2YgbGluZSBlcW5zLlxuICB2YXIgZGVub207XG5cbiAgYTEgPSB5MiAtIHkxO1xuICBiMSA9IHgxIC0geDI7XG4gIGMxID0geDIgKiB5MSAtIHgxICogeTI7ICAvLyB7IGExKnggKyBiMSp5ICsgYzEgPSAwIGlzIGxpbmUgMSB9XG5cbiAgYTIgPSB5NCAtIHkzO1xuICBiMiA9IHgzIC0geDQ7XG4gIGMyID0geDQgKiB5MyAtIHgzICogeTQ7ICAvLyB7IGEyKnggKyBiMip5ICsgYzIgPSAwIGlzIGxpbmUgMiB9XG5cbiAgZGVub20gPSBhMSAqIGIyIC0gYTIgKiBiMTtcblxuICBpZiAoZGVub20gPT0gMClcbiAge1xuICAgIHJldHVybiBudWxsO1xuICB9XG5cbiAgeCA9IChiMSAqIGMyIC0gYjIgKiBjMSkgLyBkZW5vbTtcbiAgeSA9IChhMiAqIGMxIC0gYTEgKiBjMikgLyBkZW5vbTtcblxuICByZXR1cm4gbmV3IFBvaW50KHgsIHkpO1xufVxuXG4vLyAtLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuLy8gU2VjdGlvbjogQ2xhc3MgQ29uc3RhbnRzXG4vLyAtLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuLyoqXG4gKiBTb21lIHVzZWZ1bCBwcmUtY2FsY3VsYXRlZCBjb25zdGFudHNcbiAqL1xuSUdlb21ldHJ5LkhBTEZfUEkgPSAwLjUgKiBNYXRoLlBJO1xuSUdlb21ldHJ5Lk9ORV9BTkRfSEFMRl9QSSA9IDEuNSAqIE1hdGguUEk7XG5JR2VvbWV0cnkuVFdPX1BJID0gMi4wICogTWF0aC5QSTtcbklHZW9tZXRyeS5USFJFRV9QSSA9IDMuMCAqIE1hdGguUEk7XG5cbm1vZHVsZS5leHBvcnRzID0gSUdlb21ldHJ5O1xuIiwiZnVuY3Rpb24gSU1hdGgoKSB7XG59XG5cbi8qKlxuICogVGhpcyBtZXRob2QgcmV0dXJucyB0aGUgc2lnbiBvZiB0aGUgaW5wdXQgdmFsdWUuXG4gKi9cbklNYXRoLnNpZ24gPSBmdW5jdGlvbiAodmFsdWUpIHtcbiAgaWYgKHZhbHVlID4gMClcbiAge1xuICAgIHJldHVybiAxO1xuICB9XG4gIGVsc2UgaWYgKHZhbHVlIDwgMClcbiAge1xuICAgIHJldHVybiAtMTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICByZXR1cm4gMDtcbiAgfVxufVxuXG5JTWF0aC5mbG9vciA9IGZ1bmN0aW9uICh2YWx1ZSkge1xuICByZXR1cm4gdmFsdWUgPCAwID8gTWF0aC5jZWlsKHZhbHVlKSA6IE1hdGguZmxvb3IodmFsdWUpO1xufVxuXG5JTWF0aC5jZWlsID0gZnVuY3Rpb24gKHZhbHVlKSB7XG4gIHJldHVybiB2YWx1ZSA8IDAgPyBNYXRoLmZsb29yKHZhbHVlKSA6IE1hdGguY2VpbCh2YWx1ZSk7XG59XG5cbm1vZHVsZS5leHBvcnRzID0gSU1hdGg7XG4iLCJmdW5jdGlvbiBJbnRlZ2VyKCkge1xufVxuXG5JbnRlZ2VyLk1BWF9WQUxVRSA9IDIxNDc0ODM2NDc7XG5JbnRlZ2VyLk1JTl9WQUxVRSA9IC0yMTQ3NDgzNjQ4O1xuXG5tb2R1bGUuZXhwb3J0cyA9IEludGVnZXI7XG4iLCJ2YXIgTEdyYXBoT2JqZWN0ID0gcmVxdWlyZSgnLi9MR3JhcGhPYmplY3QnKTtcblxuZnVuY3Rpb24gTEVkZ2Uoc291cmNlLCB0YXJnZXQsIHZFZGdlKSB7XG4gIExHcmFwaE9iamVjdC5jYWxsKHRoaXMsIHZFZGdlKTtcblxuICB0aGlzLmlzT3ZlcmxhcGluZ1NvdXJjZUFuZFRhcmdldCA9IGZhbHNlO1xuICB0aGlzLnZHcmFwaE9iamVjdCA9IHZFZGdlO1xuICB0aGlzLmJlbmRwb2ludHMgPSBbXTtcbiAgdGhpcy5zb3VyY2UgPSBzb3VyY2U7XG4gIHRoaXMudGFyZ2V0ID0gdGFyZ2V0O1xufVxuXG5MRWRnZS5wcm90b3R5cGUgPSBPYmplY3QuY3JlYXRlKExHcmFwaE9iamVjdC5wcm90b3R5cGUpO1xuXG5mb3IgKHZhciBwcm9wIGluIExHcmFwaE9iamVjdCkge1xuICBMRWRnZVtwcm9wXSA9IExHcmFwaE9iamVjdFtwcm9wXTtcbn1cblxuTEVkZ2UucHJvdG90eXBlLmdldFNvdXJjZSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnNvdXJjZTtcbn07XG5cbkxFZGdlLnByb3RvdHlwZS5nZXRUYXJnZXQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy50YXJnZXQ7XG59O1xuXG5MRWRnZS5wcm90b3R5cGUuaXNJbnRlckdyYXBoID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuaXNJbnRlckdyYXBoO1xufTtcblxuTEVkZ2UucHJvdG90eXBlLmdldExlbmd0aCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmxlbmd0aDtcbn07XG5cbkxFZGdlLnByb3RvdHlwZS5pc092ZXJsYXBpbmdTb3VyY2VBbmRUYXJnZXQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5pc092ZXJsYXBpbmdTb3VyY2VBbmRUYXJnZXQ7XG59O1xuXG5MRWRnZS5wcm90b3R5cGUuZ2V0QmVuZHBvaW50cyA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmJlbmRwb2ludHM7XG59O1xuXG5MRWRnZS5wcm90b3R5cGUuZ2V0TGNhID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMubGNhO1xufTtcblxuTEVkZ2UucHJvdG90eXBlLmdldFNvdXJjZUluTGNhID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuc291cmNlSW5MY2E7XG59O1xuXG5MRWRnZS5wcm90b3R5cGUuZ2V0VGFyZ2V0SW5MY2EgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy50YXJnZXRJbkxjYTtcbn07XG5cbkxFZGdlLnByb3RvdHlwZS5nZXRPdGhlckVuZCA9IGZ1bmN0aW9uIChub2RlKVxue1xuICBpZiAodGhpcy5zb3VyY2UgPT09IG5vZGUpXG4gIHtcbiAgICByZXR1cm4gdGhpcy50YXJnZXQ7XG4gIH1cbiAgZWxzZSBpZiAodGhpcy50YXJnZXQgPT09IG5vZGUpXG4gIHtcbiAgICByZXR1cm4gdGhpcy5zb3VyY2U7XG4gIH1cbiAgZWxzZVxuICB7XG4gICAgdGhyb3cgXCJOb2RlIGlzIG5vdCBpbmNpZGVudCB3aXRoIHRoaXMgZWRnZVwiO1xuICB9XG59XG5cbkxFZGdlLnByb3RvdHlwZS5nZXRPdGhlckVuZEluR3JhcGggPSBmdW5jdGlvbiAobm9kZSwgZ3JhcGgpXG57XG4gIHZhciBvdGhlckVuZCA9IHRoaXMuZ2V0T3RoZXJFbmQobm9kZSk7XG4gIHZhciByb290ID0gZ3JhcGguZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0Um9vdCgpO1xuXG4gIHdoaWxlICh0cnVlKVxuICB7XG4gICAgaWYgKG90aGVyRW5kLmdldE93bmVyKCkgPT0gZ3JhcGgpXG4gICAge1xuICAgICAgcmV0dXJuIG90aGVyRW5kO1xuICAgIH1cblxuICAgIGlmIChvdGhlckVuZC5nZXRPd25lcigpID09IHJvb3QpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuXG4gICAgb3RoZXJFbmQgPSBvdGhlckVuZC5nZXRPd25lcigpLmdldFBhcmVudCgpO1xuICB9XG5cbiAgcmV0dXJuIG51bGw7XG59O1xuXG5MRWRnZS5wcm90b3R5cGUudXBkYXRlTGVuZ3RoID0gZnVuY3Rpb24gKClcbntcbiAgdmFyIGNsaXBQb2ludENvb3JkaW5hdGVzID0gbmV3IEFycmF5KDQpO1xuXG4gIHRoaXMuaXNPdmVybGFwaW5nU291cmNlQW5kVGFyZ2V0ID1cbiAgICAgICAgICBJR2VvbWV0cnkuZ2V0SW50ZXJzZWN0aW9uKHRoaXMudGFyZ2V0LmdldFJlY3QoKSxcbiAgICAgICAgICAgICAgICAgIHRoaXMuc291cmNlLmdldFJlY3QoKSxcbiAgICAgICAgICAgICAgICAgIGNsaXBQb2ludENvb3JkaW5hdGVzKTtcblxuICBpZiAoIXRoaXMuaXNPdmVybGFwaW5nU291cmNlQW5kVGFyZ2V0KVxuICB7XG4gICAgdGhpcy5sZW5ndGhYID0gY2xpcFBvaW50Q29vcmRpbmF0ZXNbMF0gLSBjbGlwUG9pbnRDb29yZGluYXRlc1syXTtcbiAgICB0aGlzLmxlbmd0aFkgPSBjbGlwUG9pbnRDb29yZGluYXRlc1sxXSAtIGNsaXBQb2ludENvb3JkaW5hdGVzWzNdO1xuXG4gICAgaWYgKE1hdGguYWJzKHRoaXMubGVuZ3RoWCkgPCAxLjApXG4gICAge1xuICAgICAgdGhpcy5sZW5ndGhYID0gSU1hdGguc2lnbih0aGlzLmxlbmd0aFgpO1xuICAgIH1cblxuICAgIGlmIChNYXRoLmFicyh0aGlzLmxlbmd0aFkpIDwgMS4wKVxuICAgIHtcbiAgICAgIHRoaXMubGVuZ3RoWSA9IElNYXRoLnNpZ24odGhpcy5sZW5ndGhZKTtcbiAgICB9XG5cbiAgICB0aGlzLmxlbmd0aCA9IE1hdGguc3FydChcbiAgICAgICAgICAgIHRoaXMubGVuZ3RoWCAqIHRoaXMubGVuZ3RoWCArIHRoaXMubGVuZ3RoWSAqIHRoaXMubGVuZ3RoWSk7XG4gIH1cbn07XG5cbkxFZGdlLnByb3RvdHlwZS51cGRhdGVMZW5ndGhTaW1wbGUgPSBmdW5jdGlvbiAoKVxue1xuICB0aGlzLmxlbmd0aFggPSB0aGlzLnRhcmdldC5nZXRDZW50ZXJYKCkgLSB0aGlzLnNvdXJjZS5nZXRDZW50ZXJYKCk7XG4gIHRoaXMubGVuZ3RoWSA9IHRoaXMudGFyZ2V0LmdldENlbnRlclkoKSAtIHRoaXMuc291cmNlLmdldENlbnRlclkoKTtcblxuICBpZiAoTWF0aC5hYnModGhpcy5sZW5ndGhYKSA8IDEuMClcbiAge1xuICAgIHRoaXMubGVuZ3RoWCA9IElNYXRoLnNpZ24odGhpcy5sZW5ndGhYKTtcbiAgfVxuXG4gIGlmIChNYXRoLmFicyh0aGlzLmxlbmd0aFkpIDwgMS4wKVxuICB7XG4gICAgdGhpcy5sZW5ndGhZID0gSU1hdGguc2lnbih0aGlzLmxlbmd0aFkpO1xuICB9XG5cbiAgdGhpcy5sZW5ndGggPSBNYXRoLnNxcnQoXG4gICAgICAgICAgdGhpcy5sZW5ndGhYICogdGhpcy5sZW5ndGhYICsgdGhpcy5sZW5ndGhZICogdGhpcy5sZW5ndGhZKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBMRWRnZTtcbiIsInZhciBMR3JhcGhPYmplY3QgPSByZXF1aXJlKCcuL0xHcmFwaE9iamVjdCcpO1xudmFyIEludGVnZXIgPSByZXF1aXJlKCcuL0ludGVnZXInKTtcbnZhciBMYXlvdXRDb25zdGFudHMgPSByZXF1aXJlKCcuL0xheW91dENvbnN0YW50cycpO1xudmFyIExHcmFwaE1hbmFnZXIgPSByZXF1aXJlKCcuL0xHcmFwaE1hbmFnZXInKTtcbnZhciBMTm9kZSA9IHJlcXVpcmUoJy4vTE5vZGUnKTtcblxuZnVuY3Rpb24gTEdyYXBoKHBhcmVudCwgb2JqMiwgdkdyYXBoKSB7XG4gIExHcmFwaE9iamVjdC5jYWxsKHRoaXMsIHZHcmFwaCk7XG4gIHRoaXMuZXN0aW1hdGVkU2l6ZSA9IEludGVnZXIuTUlOX1ZBTFVFO1xuICB0aGlzLm1hcmdpbiA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX0dSQVBIX01BUkdJTjtcbiAgdGhpcy5lZGdlcyA9IFtdO1xuICB0aGlzLm5vZGVzID0gW107XG4gIHRoaXMuaXNDb25uZWN0ZWQgPSBmYWxzZTtcbiAgdGhpcy5wYXJlbnQgPSBwYXJlbnQ7XG5cbiAgaWYgKG9iajIgIT0gbnVsbCAmJiBvYmoyIGluc3RhbmNlb2YgTEdyYXBoTWFuYWdlcikge1xuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyID0gb2JqMjtcbiAgfVxuICBlbHNlIGlmIChvYmoyICE9IG51bGwgJiYgb2JqMiBpbnN0YW5jZW9mIExheW91dCkge1xuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyID0gb2JqMi5ncmFwaE1hbmFnZXI7XG4gIH1cbn1cblxuTEdyYXBoLnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoTEdyYXBoT2JqZWN0LnByb3RvdHlwZSk7XG5mb3IgKHZhciBwcm9wIGluIExHcmFwaE9iamVjdCkge1xuICBMR3JhcGhbcHJvcF0gPSBMR3JhcGhPYmplY3RbcHJvcF07XG59XG5cbkxHcmFwaC5wcm90b3R5cGUuZ2V0Tm9kZXMgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiB0aGlzLm5vZGVzO1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRFZGdlcyA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIHRoaXMuZWRnZXM7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmdldEdyYXBoTWFuYWdlciA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmdyYXBoTWFuYWdlcjtcbn07XG5cbkxHcmFwaC5wcm90b3R5cGUuZ2V0UGFyZW50ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMucGFyZW50O1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRMZWZ0ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMubGVmdDtcbn07XG5cbkxHcmFwaC5wcm90b3R5cGUuZ2V0UmlnaHQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yaWdodDtcbn07XG5cbkxHcmFwaC5wcm90b3R5cGUuZ2V0VG9wID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMudG9wO1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRCb3R0b20gPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5ib3R0b207XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmlzQ29ubmVjdGVkID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuaXNDb25uZWN0ZWQ7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLmFkZCA9IGZ1bmN0aW9uIChvYmoxLCBzb3VyY2VOb2RlLCB0YXJnZXROb2RlKSB7XG4gIGlmIChzb3VyY2VOb2RlID09IG51bGwgJiYgdGFyZ2V0Tm9kZSA9PSBudWxsKSB7XG4gICAgdmFyIG5ld05vZGUgPSBvYmoxO1xuICAgIGlmICh0aGlzLmdyYXBoTWFuYWdlciA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcIkdyYXBoIGhhcyBubyBncmFwaCBtZ3IhXCI7XG4gICAgfVxuICAgIGlmICh0aGlzLmdldE5vZGVzKCkuaW5kZXhPZihuZXdOb2RlKSA+IC0xKSB7XG4gICAgICB0aHJvdyBcIk5vZGUgYWxyZWFkeSBpbiBncmFwaCFcIjtcbiAgICB9XG4gICAgbmV3Tm9kZS5vd25lciA9IHRoaXM7XG4gICAgdGhpcy5nZXROb2RlcygpLnB1c2gobmV3Tm9kZSk7XG5cbiAgICByZXR1cm4gbmV3Tm9kZTtcbiAgfVxuICBlbHNlIHtcbiAgICB2YXIgbmV3RWRnZSA9IG9iajE7XG4gICAgaWYgKCEodGhpcy5nZXROb2RlcygpLmluZGV4T2Yoc291cmNlTm9kZSkgPiAtMSAmJiAodGhpcy5nZXROb2RlcygpLmluZGV4T2YodGFyZ2V0Tm9kZSkpID4gLTEpKSB7XG4gICAgICB0aHJvdyBcIlNvdXJjZSBvciB0YXJnZXQgbm90IGluIGdyYXBoIVwiO1xuICAgIH1cblxuICAgIGlmICghKHNvdXJjZU5vZGUub3duZXIgPT0gdGFyZ2V0Tm9kZS5vd25lciAmJiBzb3VyY2VOb2RlLm93bmVyID09IHRoaXMpKSB7XG4gICAgICB0aHJvdyBcIkJvdGggb3duZXJzIG11c3QgYmUgdGhpcyBncmFwaCFcIjtcbiAgICB9XG5cbiAgICBpZiAoc291cmNlTm9kZS5vd25lciAhPSB0YXJnZXROb2RlLm93bmVyKVxuICAgIHtcbiAgICAgIHJldHVybiBudWxsO1xuICAgIH1cblxuICAgIC8vIHNldCBzb3VyY2UgYW5kIHRhcmdldFxuICAgIG5ld0VkZ2Uuc291cmNlID0gc291cmNlTm9kZTtcbiAgICBuZXdFZGdlLnRhcmdldCA9IHRhcmdldE5vZGU7XG5cbiAgICAvLyBzZXQgYXMgaW50cmEtZ3JhcGggZWRnZVxuICAgIG5ld0VkZ2UuaXNJbnRlckdyYXBoID0gZmFsc2U7XG5cbiAgICAvLyBhZGQgdG8gZ3JhcGggZWRnZSBsaXN0XG4gICAgdGhpcy5nZXRFZGdlcygpLnB1c2gobmV3RWRnZSk7XG5cbiAgICAvLyBhZGQgdG8gaW5jaWRlbmN5IGxpc3RzXG4gICAgc291cmNlTm9kZS5lZGdlcy5wdXNoKG5ld0VkZ2UpO1xuXG4gICAgaWYgKHRhcmdldE5vZGUgIT0gc291cmNlTm9kZSlcbiAgICB7XG4gICAgICB0YXJnZXROb2RlLmVkZ2VzLnB1c2gobmV3RWRnZSk7XG4gICAgfVxuXG4gICAgcmV0dXJuIG5ld0VkZ2U7XG4gIH1cbn07XG5cbkxHcmFwaC5wcm90b3R5cGUucmVtb3ZlID0gZnVuY3Rpb24gKG9iaikge1xuICB2YXIgbm9kZSA9IG9iajtcbiAgaWYgKG9iaiBpbnN0YW5jZW9mIExOb2RlKSB7XG4gICAgaWYgKG5vZGUgPT0gbnVsbCkge1xuICAgICAgdGhyb3cgXCJOb2RlIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmICghKG5vZGUub3duZXIgIT0gbnVsbCAmJiBub2RlLm93bmVyID09IHRoaXMpKSB7XG4gICAgICB0aHJvdyBcIk93bmVyIGdyYXBoIGlzIGludmFsaWQhXCI7XG4gICAgfVxuICAgIGlmICh0aGlzLmdyYXBoTWFuYWdlciA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcIk93bmVyIGdyYXBoIG1hbmFnZXIgaXMgaW52YWxpZCFcIjtcbiAgICB9XG4gICAgLy8gcmVtb3ZlIGluY2lkZW50IGVkZ2VzIGZpcnN0IChtYWtlIGEgY29weSB0byBkbyBpdCBzYWZlbHkpXG4gICAgdmFyIGVkZ2VzVG9CZVJlbW92ZWQgPSBub2RlLmVkZ2VzLnNsaWNlKCk7XG4gICAgdmFyIGVkZ2U7XG4gICAgdmFyIHMgPSBlZGdlc1RvQmVSZW1vdmVkLmxlbmd0aDtcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAgICB7XG4gICAgICBlZGdlID0gZWRnZXNUb0JlUmVtb3ZlZFtpXTtcblxuICAgICAgaWYgKGVkZ2UuaXNJbnRlckdyYXBoKVxuICAgICAge1xuICAgICAgICB0aGlzLmdyYXBoTWFuYWdlci5yZW1vdmUoZWRnZSk7XG4gICAgICB9XG4gICAgICBlbHNlXG4gICAgICB7XG4gICAgICAgIGVkZ2Uuc291cmNlLm93bmVyLnJlbW92ZShlZGdlKTtcbiAgICAgIH1cbiAgICB9XG5cbiAgICAvLyBub3cgdGhlIG5vZGUgaXRzZWxmXG4gICAgdmFyIGluZGV4ID0gdGhpcy5ub2Rlcy5pbmRleE9mKG5vZGUpO1xuICAgIGlmIChpbmRleCA9PSAtMSkge1xuICAgICAgdGhyb3cgXCJOb2RlIG5vdCBpbiBvd25lciBub2RlIGxpc3QhXCI7XG4gICAgfVxuXG4gICAgdGhpcy5ub2Rlcy5zcGxpY2UoaW5kZXgsIDEpO1xuICB9XG4gIGVsc2UgaWYgKG9iaiBpbnN0YW5jZW9mIExFZGdlKSB7XG4gICAgdmFyIGVkZ2UgPSBvYmo7XG4gICAgaWYgKGVkZ2UgPT0gbnVsbCkge1xuICAgICAgdGhyb3cgXCJFZGdlIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmICghKGVkZ2Uuc291cmNlICE9IG51bGwgJiYgZWRnZS50YXJnZXQgIT0gbnVsbCkpIHtcbiAgICAgIHRocm93IFwiU291cmNlIGFuZC9vciB0YXJnZXQgaXMgbnVsbCFcIjtcbiAgICB9XG4gICAgaWYgKCEoZWRnZS5zb3VyY2Uub3duZXIgIT0gbnVsbCAmJiBlZGdlLnRhcmdldC5vd25lciAhPSBudWxsICYmXG4gICAgICAgICAgICBlZGdlLnNvdXJjZS5vd25lciA9PSB0aGlzICYmIGVkZ2UudGFyZ2V0Lm93bmVyID09IHRoaXMpKSB7XG4gICAgICB0aHJvdyBcIlNvdXJjZSBhbmQvb3IgdGFyZ2V0IG93bmVyIGlzIGludmFsaWQhXCI7XG4gICAgfVxuXG4gICAgdmFyIHNvdXJjZUluZGV4ID0gZWRnZS5zb3VyY2UuZWRnZXMuaW5kZXhPZihlZGdlKTtcbiAgICB2YXIgdGFyZ2V0SW5kZXggPSBlZGdlLnRhcmdldC5lZGdlcy5pbmRleE9mKGVkZ2UpO1xuICAgIGlmICghKHNvdXJjZUluZGV4ID4gLTEgJiYgdGFyZ2V0SW5kZXggPiAtMSkpIHtcbiAgICAgIHRocm93IFwiU291cmNlIGFuZC9vciB0YXJnZXQgZG9lc24ndCBrbm93IHRoaXMgZWRnZSFcIjtcbiAgICB9XG5cbiAgICBlZGdlLnNvdXJjZS5lZGdlcy5zcGxpY2Uoc291cmNlSW5kZXgsIDEpO1xuXG4gICAgaWYgKGVkZ2UudGFyZ2V0ICE9IGVkZ2Uuc291cmNlKVxuICAgIHtcbiAgICAgIGVkZ2UudGFyZ2V0LmVkZ2VzLnNwbGljZSh0YXJnZXRJbmRleCwgMSk7XG4gICAgfVxuXG4gICAgdmFyIGluZGV4ID0gZWRnZS5zb3VyY2Uub3duZXIuZ2V0RWRnZXMoKS5pbmRleE9mKGVkZ2UpO1xuICAgIGlmIChpbmRleCA9PSAtMSkge1xuICAgICAgdGhyb3cgXCJOb3QgaW4gb3duZXIncyBlZGdlIGxpc3QhXCI7XG4gICAgfVxuXG4gICAgZWRnZS5zb3VyY2Uub3duZXIuZ2V0RWRnZXMoKS5zcGxpY2UoaW5kZXgsIDEpO1xuICB9XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLnVwZGF0ZUxlZnRUb3AgPSBmdW5jdGlvbiAoKVxue1xuICB2YXIgdG9wID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gIHZhciBsZWZ0ID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gIHZhciBub2RlVG9wO1xuICB2YXIgbm9kZUxlZnQ7XG5cbiAgdmFyIG5vZGVzID0gdGhpcy5nZXROb2RlcygpO1xuICB2YXIgcyA9IG5vZGVzLmxlbmd0aDtcblxuICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAge1xuICAgIHZhciBsTm9kZSA9IG5vZGVzW2ldO1xuICAgIG5vZGVUb3AgPSBNYXRoLmZsb29yKGxOb2RlLmdldFRvcCgpKTtcbiAgICBub2RlTGVmdCA9IE1hdGguZmxvb3IobE5vZGUuZ2V0TGVmdCgpKTtcblxuICAgIGlmICh0b3AgPiBub2RlVG9wKVxuICAgIHtcbiAgICAgIHRvcCA9IG5vZGVUb3A7XG4gICAgfVxuXG4gICAgaWYgKGxlZnQgPiBub2RlTGVmdClcbiAgICB7XG4gICAgICBsZWZ0ID0gbm9kZUxlZnQ7XG4gICAgfVxuICB9XG5cbiAgLy8gRG8gd2UgaGF2ZSBhbnkgbm9kZXMgaW4gdGhpcyBncmFwaD9cbiAgaWYgKHRvcCA9PSBJbnRlZ2VyLk1BWF9WQUxVRSlcbiAge1xuICAgIHJldHVybiBudWxsO1xuICB9XG5cbiAgdGhpcy5sZWZ0ID0gbGVmdCAtIHRoaXMubWFyZ2luO1xuICB0aGlzLnRvcCA9IHRvcCAtIHRoaXMubWFyZ2luO1xuXG4gIC8vIEFwcGx5IHRoZSBtYXJnaW5zIGFuZCByZXR1cm4gdGhlIHJlc3VsdFxuICByZXR1cm4gbmV3IFBvaW50KHRoaXMubGVmdCwgdGhpcy50b3ApO1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS51cGRhdGVCb3VuZHMgPSBmdW5jdGlvbiAocmVjdXJzaXZlKVxue1xuICAvLyBjYWxjdWxhdGUgYm91bmRzXG4gIHZhciBsZWZ0ID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gIHZhciByaWdodCA9IC1JbnRlZ2VyLk1BWF9WQUxVRTtcbiAgdmFyIHRvcCA9IEludGVnZXIuTUFYX1ZBTFVFO1xuICB2YXIgYm90dG9tID0gLUludGVnZXIuTUFYX1ZBTFVFO1xuICB2YXIgbm9kZUxlZnQ7XG4gIHZhciBub2RlUmlnaHQ7XG4gIHZhciBub2RlVG9wO1xuICB2YXIgbm9kZUJvdHRvbTtcblxuICB2YXIgbm9kZXMgPSB0aGlzLm5vZGVzO1xuICB2YXIgcyA9IG5vZGVzLmxlbmd0aDtcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBzOyBpKyspXG4gIHtcbiAgICB2YXIgbE5vZGUgPSBub2Rlc1tpXTtcblxuICAgIGlmIChyZWN1cnNpdmUgJiYgbE5vZGUuY2hpbGQgIT0gbnVsbClcbiAgICB7XG4gICAgICBsTm9kZS51cGRhdGVCb3VuZHMoKTtcbiAgICB9XG4gICAgbm9kZUxlZnQgPSBNYXRoLmZsb29yKGxOb2RlLmdldExlZnQoKSk7XG4gICAgbm9kZVJpZ2h0ID0gTWF0aC5mbG9vcihsTm9kZS5nZXRSaWdodCgpKTtcbiAgICBub2RlVG9wID0gTWF0aC5mbG9vcihsTm9kZS5nZXRUb3AoKSk7XG4gICAgbm9kZUJvdHRvbSA9IE1hdGguZmxvb3IobE5vZGUuZ2V0Qm90dG9tKCkpO1xuXG4gICAgaWYgKGxlZnQgPiBub2RlTGVmdClcbiAgICB7XG4gICAgICBsZWZ0ID0gbm9kZUxlZnQ7XG4gICAgfVxuXG4gICAgaWYgKHJpZ2h0IDwgbm9kZVJpZ2h0KVxuICAgIHtcbiAgICAgIHJpZ2h0ID0gbm9kZVJpZ2h0O1xuICAgIH1cblxuICAgIGlmICh0b3AgPiBub2RlVG9wKVxuICAgIHtcbiAgICAgIHRvcCA9IG5vZGVUb3A7XG4gICAgfVxuXG4gICAgaWYgKGJvdHRvbSA8IG5vZGVCb3R0b20pXG4gICAge1xuICAgICAgYm90dG9tID0gbm9kZUJvdHRvbTtcbiAgICB9XG4gIH1cblxuICB2YXIgYm91bmRpbmdSZWN0ID0gbmV3IFJlY3RhbmdsZUQobGVmdCwgdG9wLCByaWdodCAtIGxlZnQsIGJvdHRvbSAtIHRvcCk7XG4gIGlmIChsZWZ0ID09IEludGVnZXIuTUFYX1ZBTFVFKVxuICB7XG4gICAgdGhpcy5sZWZ0ID0gTWF0aC5mbG9vcih0aGlzLnBhcmVudC5nZXRMZWZ0KCkpO1xuICAgIHRoaXMucmlnaHQgPSBNYXRoLmZsb29yKHRoaXMucGFyZW50LmdldFJpZ2h0KCkpO1xuICAgIHRoaXMudG9wID0gTWF0aC5mbG9vcih0aGlzLnBhcmVudC5nZXRUb3AoKSk7XG4gICAgdGhpcy5ib3R0b20gPSBNYXRoLmZsb29yKHRoaXMucGFyZW50LmdldEJvdHRvbSgpKTtcbiAgfVxuXG4gIHRoaXMubGVmdCA9IGJvdW5kaW5nUmVjdC54IC0gdGhpcy5tYXJnaW47XG4gIHRoaXMucmlnaHQgPSBib3VuZGluZ1JlY3QueCArIGJvdW5kaW5nUmVjdC53aWR0aCArIHRoaXMubWFyZ2luO1xuICB0aGlzLnRvcCA9IGJvdW5kaW5nUmVjdC55IC0gdGhpcy5tYXJnaW47XG4gIHRoaXMuYm90dG9tID0gYm91bmRpbmdSZWN0LnkgKyBib3VuZGluZ1JlY3QuaGVpZ2h0ICsgdGhpcy5tYXJnaW47XG59O1xuXG5MR3JhcGguY2FsY3VsYXRlQm91bmRzID0gZnVuY3Rpb24gKG5vZGVzKVxue1xuICB2YXIgbGVmdCA9IEludGVnZXIuTUFYX1ZBTFVFO1xuICB2YXIgcmlnaHQgPSAtSW50ZWdlci5NQVhfVkFMVUU7XG4gIHZhciB0b3AgPSBJbnRlZ2VyLk1BWF9WQUxVRTtcbiAgdmFyIGJvdHRvbSA9IC1JbnRlZ2VyLk1BWF9WQUxVRTtcbiAgdmFyIG5vZGVMZWZ0O1xuICB2YXIgbm9kZVJpZ2h0O1xuICB2YXIgbm9kZVRvcDtcbiAgdmFyIG5vZGVCb3R0b207XG5cbiAgdmFyIHMgPSBub2Rlcy5sZW5ndGg7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBzOyBpKyspXG4gIHtcbiAgICB2YXIgbE5vZGUgPSBub2Rlc1tpXTtcbiAgICBub2RlTGVmdCA9IE1hdGguZmxvb3IobE5vZGUuZ2V0TGVmdCgpKTtcbiAgICBub2RlUmlnaHQgPSBNYXRoLmZsb29yKGxOb2RlLmdldFJpZ2h0KCkpO1xuICAgIG5vZGVUb3AgPSBNYXRoLmZsb29yKGxOb2RlLmdldFRvcCgpKTtcbiAgICBub2RlQm90dG9tID0gTWF0aC5mbG9vcihsTm9kZS5nZXRCb3R0b20oKSk7XG5cbiAgICBpZiAobGVmdCA+IG5vZGVMZWZ0KVxuICAgIHtcbiAgICAgIGxlZnQgPSBub2RlTGVmdDtcbiAgICB9XG5cbiAgICBpZiAocmlnaHQgPCBub2RlUmlnaHQpXG4gICAge1xuICAgICAgcmlnaHQgPSBub2RlUmlnaHQ7XG4gICAgfVxuXG4gICAgaWYgKHRvcCA+IG5vZGVUb3ApXG4gICAge1xuICAgICAgdG9wID0gbm9kZVRvcDtcbiAgICB9XG5cbiAgICBpZiAoYm90dG9tIDwgbm9kZUJvdHRvbSlcbiAgICB7XG4gICAgICBib3R0b20gPSBub2RlQm90dG9tO1xuICAgIH1cbiAgfVxuXG4gIHZhciBib3VuZGluZ1JlY3QgPSBuZXcgUmVjdGFuZ2xlRChsZWZ0LCB0b3AsIHJpZ2h0IC0gbGVmdCwgYm90dG9tIC0gdG9wKTtcblxuICByZXR1cm4gYm91bmRpbmdSZWN0O1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5nZXRJbmNsdXNpb25UcmVlRGVwdGggPSBmdW5jdGlvbiAoKVxue1xuICBpZiAodGhpcyA9PSB0aGlzLmdyYXBoTWFuYWdlci5nZXRSb290KCkpXG4gIHtcbiAgICByZXR1cm4gMTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICByZXR1cm4gdGhpcy5wYXJlbnQuZ2V0SW5jbHVzaW9uVHJlZURlcHRoKCk7XG4gIH1cbn07XG5cbkxHcmFwaC5wcm90b3R5cGUuZ2V0RXN0aW1hdGVkU2l6ZSA9IGZ1bmN0aW9uICgpXG57XG4gIGlmICh0aGlzLmVzdGltYXRlZFNpemUgPT0gSW50ZWdlci5NSU5fVkFMVUUpIHtcbiAgICB0aHJvdyBcImFzc2VydCBmYWlsZWRcIjtcbiAgfVxuICByZXR1cm4gdGhpcy5lc3RpbWF0ZWRTaXplO1xufTtcblxuTEdyYXBoLnByb3RvdHlwZS5jYWxjRXN0aW1hdGVkU2l6ZSA9IGZ1bmN0aW9uICgpXG57XG4gIHZhciBzaXplID0gMDtcbiAgdmFyIG5vZGVzID0gdGhpcy5ub2RlcztcbiAgdmFyIHMgPSBub2Rlcy5sZW5ndGg7XG5cbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBzOyBpKyspXG4gIHtcbiAgICB2YXIgbE5vZGUgPSBub2Rlc1tpXTtcbiAgICBzaXplICs9IGxOb2RlLmNhbGNFc3RpbWF0ZWRTaXplKCk7XG4gIH1cblxuICBpZiAoc2l6ZSA9PSAwKVxuICB7XG4gICAgdGhpcy5lc3RpbWF0ZWRTaXplID0gTGF5b3V0Q29uc3RhbnRzLkVNUFRZX0NPTVBPVU5EX05PREVfU0laRTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICB0aGlzLmVzdGltYXRlZFNpemUgPSBNYXRoLmZsb29yKHNpemUgLyBNYXRoLnNxcnQodGhpcy5ub2Rlcy5sZW5ndGgpKTtcbiAgfVxuXG4gIHJldHVybiBNYXRoLmZsb29yKHRoaXMuZXN0aW1hdGVkU2l6ZSk7XG59O1xuXG5MR3JhcGgucHJvdG90eXBlLnVwZGF0ZUNvbm5lY3RlZCA9IGZ1bmN0aW9uICgpXG57XG4gIGlmICh0aGlzLm5vZGVzLmxlbmd0aCA9PSAwKVxuICB7XG4gICAgdGhpcy5pc0Nvbm5lY3RlZCA9IHRydWU7XG4gICAgcmV0dXJuO1xuICB9XG5cbiAgdmFyIHRvQmVWaXNpdGVkID0gW107XG4gIHZhciB2aXNpdGVkID0gbmV3IEhhc2hTZXQoKTtcbiAgdmFyIGN1cnJlbnROb2RlID0gdGhpcy5ub2Rlc1swXTtcbiAgdmFyIG5laWdoYm9yRWRnZXM7XG4gIHZhciBjdXJyZW50TmVpZ2hib3I7XG4gIHRvQmVWaXNpdGVkID0gdG9CZVZpc2l0ZWQuY29uY2F0KGN1cnJlbnROb2RlLndpdGhDaGlsZHJlbigpKTtcblxuICB3aGlsZSAodG9CZVZpc2l0ZWQubGVuZ3RoID4gMClcbiAge1xuICAgIGN1cnJlbnROb2RlID0gdG9CZVZpc2l0ZWQuc2hpZnQoKTtcbiAgICB2aXNpdGVkLmFkZChjdXJyZW50Tm9kZSk7XG5cbiAgICAvLyBUcmF2ZXJzZSBhbGwgbmVpZ2hib3JzIG9mIHRoaXMgbm9kZVxuICAgIG5laWdoYm9yRWRnZXMgPSBjdXJyZW50Tm9kZS5nZXRFZGdlcygpO1xuICAgIHZhciBzID0gbmVpZ2hib3JFZGdlcy5sZW5ndGg7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBzOyBpKyspXG4gICAge1xuICAgICAgdmFyIG5laWdoYm9yRWRnZSA9IG5laWdoYm9yRWRnZXNbaV07XG4gICAgICBcbiAgICAgIGN1cnJlbnROZWlnaGJvciA9XG4gICAgICAgICAgICAgIG5laWdoYm9yRWRnZS5nZXRPdGhlckVuZEluR3JhcGgoY3VycmVudE5vZGUsIHRoaXMpO1xuXG4gICAgICAvLyBBZGQgdW52aXNpdGVkIG5laWdoYm9ycyB0byB0aGUgbGlzdCB0byB2aXNpdFxuICAgICAgaWYgKGN1cnJlbnROZWlnaGJvciAhPSBudWxsICYmXG4gICAgICAgICAgICAgICF2aXNpdGVkLmNvbnRhaW5zKGN1cnJlbnROZWlnaGJvcikpXG4gICAgICB7XG4gICAgICAgIHRvQmVWaXNpdGVkID0gdG9CZVZpc2l0ZWQuY29uY2F0KGN1cnJlbnROZWlnaGJvci53aXRoQ2hpbGRyZW4oKSk7XG4gICAgICB9XG4gICAgfVxuICB9XG5cbiAgdGhpcy5pc0Nvbm5lY3RlZCA9IGZhbHNlO1xuXG4gIGlmICh2aXNpdGVkLnNpemUoKSA+PSB0aGlzLm5vZGVzLmxlbmd0aClcbiAge1xuICAgIHZhciBub09mVmlzaXRlZEluVGhpc0dyYXBoID0gMDtcblxuICAgIHZhciBzID0gdmlzaXRlZC5zaXplKCk7XG4gICAgZm9yICh2YXIgdmlzaXRlZElkIGluIHZpc2l0ZWQuc2V0KVxuICAgIHtcbiAgICAgIHZhciB2aXNpdGVkTm9kZSA9IHZpc2l0ZWQuc2V0W3Zpc2l0ZWRJZF07XG4gICAgICBpZiAodmlzaXRlZE5vZGUub3duZXIgPT0gdGhpcylcbiAgICAgIHtcbiAgICAgICAgbm9PZlZpc2l0ZWRJblRoaXNHcmFwaCsrO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChub09mVmlzaXRlZEluVGhpc0dyYXBoID09IHRoaXMubm9kZXMubGVuZ3RoKVxuICAgIHtcbiAgICAgIHRoaXMuaXNDb25uZWN0ZWQgPSB0cnVlO1xuICAgIH1cbiAgfVxufTtcblxubW9kdWxlLmV4cG9ydHMgPSBMR3JhcGg7XG4iLCJmdW5jdGlvbiBMR3JhcGhNYW5hZ2VyKGxheW91dCkge1xuICB0aGlzLmxheW91dCA9IGxheW91dDtcblxuICB0aGlzLmdyYXBocyA9IFtdO1xuICB0aGlzLmVkZ2VzID0gW107XG59XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmFkZFJvb3QgPSBmdW5jdGlvbiAoKVxue1xuICB2YXIgbmdyYXBoID0gdGhpcy5sYXlvdXQubmV3R3JhcGgoKTtcbiAgdmFyIG5ub2RlID0gdGhpcy5sYXlvdXQubmV3Tm9kZShudWxsKTtcbiAgdmFyIHJvb3QgPSB0aGlzLmFkZChuZ3JhcGgsIG5ub2RlKTtcbiAgdGhpcy5zZXRSb290R3JhcGgocm9vdCk7XG4gIHJldHVybiB0aGlzLnJvb3RHcmFwaDtcbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmFkZCA9IGZ1bmN0aW9uIChuZXdHcmFwaCwgcGFyZW50Tm9kZSwgbmV3RWRnZSwgc291cmNlTm9kZSwgdGFyZ2V0Tm9kZSlcbntcbiAgLy90aGVyZSBhcmUganVzdCAyIHBhcmFtZXRlcnMgYXJlIHBhc3NlZCB0aGVuIGl0IGFkZHMgYW4gTEdyYXBoIGVsc2UgaXQgYWRkcyBhbiBMRWRnZVxuICBpZiAobmV3RWRnZSA9PSBudWxsICYmIHNvdXJjZU5vZGUgPT0gbnVsbCAmJiB0YXJnZXROb2RlID09IG51bGwpIHtcbiAgICBpZiAobmV3R3JhcGggPT0gbnVsbCkge1xuICAgICAgdGhyb3cgXCJHcmFwaCBpcyBudWxsIVwiO1xuICAgIH1cbiAgICBpZiAocGFyZW50Tm9kZSA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcIlBhcmVudCBub2RlIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmICh0aGlzLmdyYXBocy5pbmRleE9mKG5ld0dyYXBoKSA+IC0xKSB7XG4gICAgICB0aHJvdyBcIkdyYXBoIGFscmVhZHkgaW4gdGhpcyBncmFwaCBtZ3IhXCI7XG4gICAgfVxuXG4gICAgdGhpcy5ncmFwaHMucHVzaChuZXdHcmFwaCk7XG5cbiAgICBpZiAobmV3R3JhcGgucGFyZW50ICE9IG51bGwpIHtcbiAgICAgIHRocm93IFwiQWxyZWFkeSBoYXMgYSBwYXJlbnQhXCI7XG4gICAgfVxuICAgIGlmIChwYXJlbnROb2RlLmNoaWxkICE9IG51bGwpIHtcbiAgICAgIHRocm93ICBcIkFscmVhZHkgaGFzIGEgY2hpbGQhXCI7XG4gICAgfVxuXG4gICAgbmV3R3JhcGgucGFyZW50ID0gcGFyZW50Tm9kZTtcbiAgICBwYXJlbnROb2RlLmNoaWxkID0gbmV3R3JhcGg7XG5cbiAgICByZXR1cm4gbmV3R3JhcGg7XG4gIH1cbiAgZWxzZSB7XG4gICAgLy9jaGFuZ2UgdGhlIG9yZGVyIG9mIHRoZSBwYXJhbWV0ZXJzXG4gICAgdGFyZ2V0Tm9kZSA9IG5ld0VkZ2U7XG4gICAgc291cmNlTm9kZSA9IHBhcmVudE5vZGU7XG4gICAgbmV3RWRnZSA9IG5ld0dyYXBoO1xuICAgIHZhciBzb3VyY2VHcmFwaCA9IHNvdXJjZU5vZGUuZ2V0T3duZXIoKTtcbiAgICB2YXIgdGFyZ2V0R3JhcGggPSB0YXJnZXROb2RlLmdldE93bmVyKCk7XG5cbiAgICBpZiAoIShzb3VyY2VHcmFwaCAhPSBudWxsICYmIHNvdXJjZUdyYXBoLmdldEdyYXBoTWFuYWdlcigpID09IHRoaXMpKSB7XG4gICAgICB0aHJvdyBcIlNvdXJjZSBub3QgaW4gdGhpcyBncmFwaCBtZ3IhXCI7XG4gICAgfVxuICAgIGlmICghKHRhcmdldEdyYXBoICE9IG51bGwgJiYgdGFyZ2V0R3JhcGguZ2V0R3JhcGhNYW5hZ2VyKCkgPT0gdGhpcykpIHtcbiAgICAgIHRocm93IFwiVGFyZ2V0IG5vdCBpbiB0aGlzIGdyYXBoIG1nciFcIjtcbiAgICB9XG5cbiAgICBpZiAoc291cmNlR3JhcGggPT0gdGFyZ2V0R3JhcGgpXG4gICAge1xuICAgICAgbmV3RWRnZS5pc0ludGVyR3JhcGggPSBmYWxzZTtcbiAgICAgIHJldHVybiBzb3VyY2VHcmFwaC5hZGQobmV3RWRnZSwgc291cmNlTm9kZSwgdGFyZ2V0Tm9kZSk7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICBuZXdFZGdlLmlzSW50ZXJHcmFwaCA9IHRydWU7XG5cbiAgICAgIC8vIHNldCBzb3VyY2UgYW5kIHRhcmdldFxuICAgICAgbmV3RWRnZS5zb3VyY2UgPSBzb3VyY2VOb2RlO1xuICAgICAgbmV3RWRnZS50YXJnZXQgPSB0YXJnZXROb2RlO1xuXG4gICAgICAvLyBhZGQgZWRnZSB0byBpbnRlci1ncmFwaCBlZGdlIGxpc3RcbiAgICAgIGlmICh0aGlzLmVkZ2VzLmluZGV4T2YobmV3RWRnZSkgPiAtMSkge1xuICAgICAgICB0aHJvdyBcIkVkZ2UgYWxyZWFkeSBpbiBpbnRlci1ncmFwaCBlZGdlIGxpc3QhXCI7XG4gICAgICB9XG5cbiAgICAgIHRoaXMuZWRnZXMucHVzaChuZXdFZGdlKTtcblxuICAgICAgLy8gYWRkIGVkZ2UgdG8gc291cmNlIGFuZCB0YXJnZXQgaW5jaWRlbmN5IGxpc3RzXG4gICAgICBpZiAoIShuZXdFZGdlLnNvdXJjZSAhPSBudWxsICYmIG5ld0VkZ2UudGFyZ2V0ICE9IG51bGwpKSB7XG4gICAgICAgIHRocm93IFwiRWRnZSBzb3VyY2UgYW5kL29yIHRhcmdldCBpcyBudWxsIVwiO1xuICAgICAgfVxuXG4gICAgICBpZiAoIShuZXdFZGdlLnNvdXJjZS5lZGdlcy5pbmRleE9mKG5ld0VkZ2UpID09IC0xICYmIG5ld0VkZ2UudGFyZ2V0LmVkZ2VzLmluZGV4T2YobmV3RWRnZSkgPT0gLTEpKSB7XG4gICAgICAgIHRocm93IFwiRWRnZSBhbHJlYWR5IGluIHNvdXJjZSBhbmQvb3IgdGFyZ2V0IGluY2lkZW5jeSBsaXN0IVwiO1xuICAgICAgfVxuXG4gICAgICBuZXdFZGdlLnNvdXJjZS5lZGdlcy5wdXNoKG5ld0VkZ2UpO1xuICAgICAgbmV3RWRnZS50YXJnZXQuZWRnZXMucHVzaChuZXdFZGdlKTtcblxuICAgICAgcmV0dXJuIG5ld0VkZ2U7XG4gICAgfVxuICB9XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5yZW1vdmUgPSBmdW5jdGlvbiAobE9iaikge1xuICBpZiAobE9iaiBpbnN0YW5jZW9mIExHcmFwaCkge1xuICAgIHZhciBncmFwaCA9IGxPYmo7XG4gICAgaWYgKGdyYXBoLmdldEdyYXBoTWFuYWdlcigpICE9IHRoaXMpIHtcbiAgICAgIHRocm93IFwiR3JhcGggbm90IGluIHRoaXMgZ3JhcGggbWdyXCI7XG4gICAgfVxuICAgIGlmICghKGdyYXBoID09IHRoaXMucm9vdEdyYXBoIHx8IChncmFwaC5wYXJlbnQgIT0gbnVsbCAmJiBncmFwaC5wYXJlbnQuZ3JhcGhNYW5hZ2VyID09IHRoaXMpKSkge1xuICAgICAgdGhyb3cgXCJJbnZhbGlkIHBhcmVudCBub2RlIVwiO1xuICAgIH1cblxuICAgIC8vIGZpcnN0IHRoZSBlZGdlcyAobWFrZSBhIGNvcHkgdG8gZG8gaXQgc2FmZWx5KVxuICAgIHZhciBlZGdlc1RvQmVSZW1vdmVkID0gW107XG5cbiAgICBlZGdlc1RvQmVSZW1vdmVkID0gZWRnZXNUb0JlUmVtb3ZlZC5jb25jYXQoZ3JhcGguZ2V0RWRnZXMoKSk7XG5cbiAgICB2YXIgZWRnZTtcbiAgICB2YXIgcyA9IGVkZ2VzVG9CZVJlbW92ZWQubGVuZ3RoO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICAgIHtcbiAgICAgIGVkZ2UgPSBlZGdlc1RvQmVSZW1vdmVkW2ldO1xuICAgICAgZ3JhcGgucmVtb3ZlKGVkZ2UpO1xuICAgIH1cblxuICAgIC8vIHRoZW4gdGhlIG5vZGVzIChtYWtlIGEgY29weSB0byBkbyBpdCBzYWZlbHkpXG4gICAgdmFyIG5vZGVzVG9CZVJlbW92ZWQgPSBbXTtcblxuICAgIG5vZGVzVG9CZVJlbW92ZWQgPSBub2Rlc1RvQmVSZW1vdmVkLmNvbmNhdChncmFwaC5nZXROb2RlcygpKTtcblxuICAgIHZhciBub2RlO1xuICAgIHMgPSBub2Rlc1RvQmVSZW1vdmVkLmxlbmd0aDtcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHM7IGkrKylcbiAgICB7XG4gICAgICBub2RlID0gbm9kZXNUb0JlUmVtb3ZlZFtpXTtcbiAgICAgIGdyYXBoLnJlbW92ZShub2RlKTtcbiAgICB9XG5cbiAgICAvLyBjaGVjayBpZiBncmFwaCBpcyB0aGUgcm9vdFxuICAgIGlmIChncmFwaCA9PSB0aGlzLnJvb3RHcmFwaClcbiAgICB7XG4gICAgICB0aGlzLnNldFJvb3RHcmFwaChudWxsKTtcbiAgICB9XG5cbiAgICAvLyBub3cgcmVtb3ZlIHRoZSBncmFwaCBpdHNlbGZcbiAgICB2YXIgaW5kZXggPSB0aGlzLmdyYXBocy5pbmRleE9mKGdyYXBoKTtcbiAgICB0aGlzLmdyYXBocy5zcGxpY2UoaW5kZXgsIDEpO1xuXG4gICAgLy8gYWxzbyByZXNldCB0aGUgcGFyZW50IG9mIHRoZSBncmFwaFxuICAgIGdyYXBoLnBhcmVudCA9IG51bGw7XG4gIH1cbiAgZWxzZSBpZiAobE9iaiBpbnN0YW5jZW9mIExFZGdlKSB7XG4gICAgZWRnZSA9IGxPYmo7XG4gICAgaWYgKGVkZ2UgPT0gbnVsbCkge1xuICAgICAgdGhyb3cgXCJFZGdlIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmICghZWRnZS5pc0ludGVyR3JhcGgpIHtcbiAgICAgIHRocm93IFwiTm90IGFuIGludGVyLWdyYXBoIGVkZ2UhXCI7XG4gICAgfVxuICAgIGlmICghKGVkZ2Uuc291cmNlICE9IG51bGwgJiYgZWRnZS50YXJnZXQgIT0gbnVsbCkpIHtcbiAgICAgIHRocm93IFwiU291cmNlIGFuZC9vciB0YXJnZXQgaXMgbnVsbCFcIjtcbiAgICB9XG5cbiAgICAvLyByZW1vdmUgZWRnZSBmcm9tIHNvdXJjZSBhbmQgdGFyZ2V0IG5vZGVzJyBpbmNpZGVuY3kgbGlzdHNcblxuICAgIGlmICghKGVkZ2Uuc291cmNlLmVkZ2VzLmluZGV4T2YoZWRnZSkgIT0gLTEgJiYgZWRnZS50YXJnZXQuZWRnZXMuaW5kZXhPZihlZGdlKSAhPSAtMSkpIHtcbiAgICAgIHRocm93IFwiU291cmNlIGFuZC9vciB0YXJnZXQgZG9lc24ndCBrbm93IHRoaXMgZWRnZSFcIjtcbiAgICB9XG5cbiAgICB2YXIgaW5kZXggPSBlZGdlLnNvdXJjZS5lZGdlcy5pbmRleE9mKGVkZ2UpO1xuICAgIGVkZ2Uuc291cmNlLmVkZ2VzLnNwbGljZShpbmRleCwgMSk7XG4gICAgaW5kZXggPSBlZGdlLnRhcmdldC5lZGdlcy5pbmRleE9mKGVkZ2UpO1xuICAgIGVkZ2UudGFyZ2V0LmVkZ2VzLnNwbGljZShpbmRleCwgMSk7XG5cbiAgICAvLyByZW1vdmUgZWRnZSBmcm9tIG93bmVyIGdyYXBoIG1hbmFnZXIncyBpbnRlci1ncmFwaCBlZGdlIGxpc3RcblxuICAgIGlmICghKGVkZ2Uuc291cmNlLm93bmVyICE9IG51bGwgJiYgZWRnZS5zb3VyY2Uub3duZXIuZ2V0R3JhcGhNYW5hZ2VyKCkgIT0gbnVsbCkpIHtcbiAgICAgIHRocm93IFwiRWRnZSBvd25lciBncmFwaCBvciBvd25lciBncmFwaCBtYW5hZ2VyIGlzIG51bGwhXCI7XG4gICAgfVxuICAgIGlmIChlZGdlLnNvdXJjZS5vd25lci5nZXRHcmFwaE1hbmFnZXIoKS5lZGdlcy5pbmRleE9mKGVkZ2UpID09IC0xKSB7XG4gICAgICB0aHJvdyBcIk5vdCBpbiBvd25lciBncmFwaCBtYW5hZ2VyJ3MgZWRnZSBsaXN0IVwiO1xuICAgIH1cblxuICAgIHZhciBpbmRleCA9IGVkZ2Uuc291cmNlLm93bmVyLmdldEdyYXBoTWFuYWdlcigpLmVkZ2VzLmluZGV4T2YoZWRnZSk7XG4gICAgZWRnZS5zb3VyY2Uub3duZXIuZ2V0R3JhcGhNYW5hZ2VyKCkuZWRnZXMuc3BsaWNlKGluZGV4LCAxKTtcbiAgfVxufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUudXBkYXRlQm91bmRzID0gZnVuY3Rpb24gKClcbntcbiAgdGhpcy5yb290R3JhcGgudXBkYXRlQm91bmRzKHRydWUpO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0R3JhcGhzID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuZ3JhcGhzO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0QWxsTm9kZXMgPSBmdW5jdGlvbiAoKVxue1xuICBpZiAodGhpcy5hbGxOb2RlcyA9PSBudWxsKVxuICB7XG4gICAgdmFyIG5vZGVMaXN0ID0gW107XG4gICAgdmFyIGdyYXBocyA9IHRoaXMuZ2V0R3JhcGhzKCk7XG4gICAgdmFyIHMgPSBncmFwaHMubGVuZ3RoO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICAgIHtcbiAgICAgIG5vZGVMaXN0ID0gbm9kZUxpc3QuY29uY2F0KGdyYXBoc1tpXS5nZXROb2RlcygpKTtcbiAgICB9XG4gICAgdGhpcy5hbGxOb2RlcyA9IG5vZGVMaXN0O1xuICB9XG4gIHJldHVybiB0aGlzLmFsbE5vZGVzO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUucmVzZXRBbGxOb2RlcyA9IGZ1bmN0aW9uICgpXG57XG4gIHRoaXMuYWxsTm9kZXMgPSBudWxsO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUucmVzZXRBbGxFZGdlcyA9IGZ1bmN0aW9uICgpXG57XG4gIHRoaXMuYWxsRWRnZXMgPSBudWxsO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbiA9IGZ1bmN0aW9uICgpXG57XG4gIHRoaXMuYWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24gPSBudWxsO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0QWxsRWRnZXMgPSBmdW5jdGlvbiAoKVxue1xuICBpZiAodGhpcy5hbGxFZGdlcyA9PSBudWxsKVxuICB7XG4gICAgdmFyIGVkZ2VMaXN0ID0gW107XG4gICAgdmFyIGdyYXBocyA9IHRoaXMuZ2V0R3JhcGhzKCk7XG4gICAgdmFyIHMgPSBncmFwaHMubGVuZ3RoO1xuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZ3JhcGhzLmxlbmd0aDsgaSsrKVxuICAgIHtcbiAgICAgIGVkZ2VMaXN0ID0gZWRnZUxpc3QuY29uY2F0KGdyYXBoc1tpXS5nZXRFZGdlcygpKTtcbiAgICB9XG5cbiAgICBlZGdlTGlzdCA9IGVkZ2VMaXN0LmNvbmNhdCh0aGlzLmVkZ2VzKTtcblxuICAgIHRoaXMuYWxsRWRnZXMgPSBlZGdlTGlzdDtcbiAgfVxuICByZXR1cm4gdGhpcy5hbGxFZGdlcztcbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmdldEFsbE5vZGVzVG9BcHBseUdyYXZpdGF0aW9uID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuYWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb247XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5zZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbiA9IGZ1bmN0aW9uIChub2RlTGlzdClcbntcbiAgaWYgKHRoaXMuYWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24gIT0gbnVsbCkge1xuICAgIHRocm93IFwiYXNzZXJ0IGZhaWxlZFwiO1xuICB9XG5cbiAgdGhpcy5hbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbiA9IG5vZGVMaXN0O1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuZ2V0Um9vdCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnJvb3RHcmFwaDtcbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLnNldFJvb3RHcmFwaCA9IGZ1bmN0aW9uIChncmFwaClcbntcbiAgaWYgKGdyYXBoLmdldEdyYXBoTWFuYWdlcigpICE9IHRoaXMpIHtcbiAgICB0aHJvdyBcIlJvb3Qgbm90IGluIHRoaXMgZ3JhcGggbWdyIVwiO1xuICB9XG5cbiAgdGhpcy5yb290R3JhcGggPSBncmFwaDtcbiAgLy8gcm9vdCBncmFwaCBtdXN0IGhhdmUgYSByb290IG5vZGUgYXNzb2NpYXRlZCB3aXRoIGl0IGZvciBjb252ZW5pZW5jZVxuICBpZiAoZ3JhcGgucGFyZW50ID09IG51bGwpXG4gIHtcbiAgICBncmFwaC5wYXJlbnQgPSB0aGlzLmxheW91dC5uZXdOb2RlKFwiUm9vdCBub2RlXCIpO1xuICB9XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5nZXRMYXlvdXQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sYXlvdXQ7XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5pc09uZUFuY2VzdG9yT2ZPdGhlciA9IGZ1bmN0aW9uIChmaXJzdE5vZGUsIHNlY29uZE5vZGUpXG57XG4gIGlmICghKGZpcnN0Tm9kZSAhPSBudWxsICYmIHNlY29uZE5vZGUgIT0gbnVsbCkpIHtcbiAgICB0aHJvdyBcImFzc2VydCBmYWlsZWRcIjtcbiAgfVxuXG4gIGlmIChmaXJzdE5vZGUgPT0gc2Vjb25kTm9kZSlcbiAge1xuICAgIHJldHVybiB0cnVlO1xuICB9XG4gIC8vIElzIHNlY29uZCBub2RlIGFuIGFuY2VzdG9yIG9mIHRoZSBmaXJzdCBvbmU/XG4gIHZhciBvd25lckdyYXBoID0gZmlyc3ROb2RlLmdldE93bmVyKCk7XG4gIHZhciBwYXJlbnROb2RlO1xuXG4gIGRvXG4gIHtcbiAgICBwYXJlbnROb2RlID0gb3duZXJHcmFwaC5nZXRQYXJlbnQoKTtcblxuICAgIGlmIChwYXJlbnROb2RlID09IG51bGwpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuXG4gICAgaWYgKHBhcmVudE5vZGUgPT0gc2Vjb25kTm9kZSlcbiAgICB7XG4gICAgICByZXR1cm4gdHJ1ZTtcbiAgICB9XG5cbiAgICBvd25lckdyYXBoID0gcGFyZW50Tm9kZS5nZXRPd25lcigpO1xuICAgIGlmIChvd25lckdyYXBoID09IG51bGwpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9IHdoaWxlICh0cnVlKTtcbiAgLy8gSXMgZmlyc3Qgbm9kZSBhbiBhbmNlc3RvciBvZiB0aGUgc2Vjb25kIG9uZT9cbiAgb3duZXJHcmFwaCA9IHNlY29uZE5vZGUuZ2V0T3duZXIoKTtcblxuICBkb1xuICB7XG4gICAgcGFyZW50Tm9kZSA9IG93bmVyR3JhcGguZ2V0UGFyZW50KCk7XG5cbiAgICBpZiAocGFyZW50Tm9kZSA9PSBudWxsKVxuICAgIHtcbiAgICAgIGJyZWFrO1xuICAgIH1cblxuICAgIGlmIChwYXJlbnROb2RlID09IGZpcnN0Tm9kZSlcbiAgICB7XG4gICAgICByZXR1cm4gdHJ1ZTtcbiAgICB9XG5cbiAgICBvd25lckdyYXBoID0gcGFyZW50Tm9kZS5nZXRPd25lcigpO1xuICAgIGlmIChvd25lckdyYXBoID09IG51bGwpXG4gICAge1xuICAgICAgYnJlYWs7XG4gICAgfVxuICB9IHdoaWxlICh0cnVlKTtcblxuICByZXR1cm4gZmFsc2U7XG59O1xuXG5MR3JhcGhNYW5hZ2VyLnByb3RvdHlwZS5jYWxjTG93ZXN0Q29tbW9uQW5jZXN0b3JzID0gZnVuY3Rpb24gKClcbntcbiAgdmFyIGVkZ2U7XG4gIHZhciBzb3VyY2VOb2RlO1xuICB2YXIgdGFyZ2V0Tm9kZTtcbiAgdmFyIHNvdXJjZUFuY2VzdG9yR3JhcGg7XG4gIHZhciB0YXJnZXRBbmNlc3RvckdyYXBoO1xuXG4gIHZhciBlZGdlcyA9IHRoaXMuZ2V0QWxsRWRnZXMoKTtcbiAgdmFyIHMgPSBlZGdlcy5sZW5ndGg7XG4gIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICB7XG4gICAgZWRnZSA9IGVkZ2VzW2ldO1xuXG4gICAgc291cmNlTm9kZSA9IGVkZ2Uuc291cmNlO1xuICAgIHRhcmdldE5vZGUgPSBlZGdlLnRhcmdldDtcbiAgICBlZGdlLmxjYSA9IG51bGw7XG4gICAgZWRnZS5zb3VyY2VJbkxjYSA9IHNvdXJjZU5vZGU7XG4gICAgZWRnZS50YXJnZXRJbkxjYSA9IHRhcmdldE5vZGU7XG5cbiAgICBpZiAoc291cmNlTm9kZSA9PSB0YXJnZXROb2RlKVxuICAgIHtcbiAgICAgIGVkZ2UubGNhID0gc291cmNlTm9kZS5nZXRPd25lcigpO1xuICAgICAgY29udGludWU7XG4gICAgfVxuXG4gICAgc291cmNlQW5jZXN0b3JHcmFwaCA9IHNvdXJjZU5vZGUuZ2V0T3duZXIoKTtcblxuICAgIHdoaWxlIChlZGdlLmxjYSA9PSBudWxsKVxuICAgIHtcbiAgICAgIHRhcmdldEFuY2VzdG9yR3JhcGggPSB0YXJnZXROb2RlLmdldE93bmVyKCk7XG5cbiAgICAgIHdoaWxlIChlZGdlLmxjYSA9PSBudWxsKVxuICAgICAge1xuICAgICAgICBpZiAodGFyZ2V0QW5jZXN0b3JHcmFwaCA9PSBzb3VyY2VBbmNlc3RvckdyYXBoKVxuICAgICAgICB7XG4gICAgICAgICAgZWRnZS5sY2EgPSB0YXJnZXRBbmNlc3RvckdyYXBoO1xuICAgICAgICAgIGJyZWFrO1xuICAgICAgICB9XG5cbiAgICAgICAgaWYgKHRhcmdldEFuY2VzdG9yR3JhcGggPT0gdGhpcy5yb290R3JhcGgpXG4gICAgICAgIHtcbiAgICAgICAgICBicmVhaztcbiAgICAgICAgfVxuXG4gICAgICAgIGlmIChlZGdlLmxjYSAhPSBudWxsKSB7XG4gICAgICAgICAgdGhyb3cgXCJhc3NlcnQgZmFpbGVkXCI7XG4gICAgICAgIH1cbiAgICAgICAgZWRnZS50YXJnZXRJbkxjYSA9IHRhcmdldEFuY2VzdG9yR3JhcGguZ2V0UGFyZW50KCk7XG4gICAgICAgIHRhcmdldEFuY2VzdG9yR3JhcGggPSBlZGdlLnRhcmdldEluTGNhLmdldE93bmVyKCk7XG4gICAgICB9XG5cbiAgICAgIGlmIChzb3VyY2VBbmNlc3RvckdyYXBoID09IHRoaXMucm9vdEdyYXBoKVxuICAgICAge1xuICAgICAgICBicmVhaztcbiAgICAgIH1cblxuICAgICAgaWYgKGVkZ2UubGNhID09IG51bGwpXG4gICAgICB7XG4gICAgICAgIGVkZ2Uuc291cmNlSW5MY2EgPSBzb3VyY2VBbmNlc3RvckdyYXBoLmdldFBhcmVudCgpO1xuICAgICAgICBzb3VyY2VBbmNlc3RvckdyYXBoID0gZWRnZS5zb3VyY2VJbkxjYS5nZXRPd25lcigpO1xuICAgICAgfVxuICAgIH1cblxuICAgIGlmIChlZGdlLmxjYSA9PSBudWxsKSB7XG4gICAgICB0aHJvdyBcImFzc2VydCBmYWlsZWRcIjtcbiAgICB9XG4gIH1cbn07XG5cbkxHcmFwaE1hbmFnZXIucHJvdG90eXBlLmNhbGNMb3dlc3RDb21tb25BbmNlc3RvciA9IGZ1bmN0aW9uIChmaXJzdE5vZGUsIHNlY29uZE5vZGUpXG57XG4gIGlmIChmaXJzdE5vZGUgPT0gc2Vjb25kTm9kZSlcbiAge1xuICAgIHJldHVybiBmaXJzdE5vZGUuZ2V0T3duZXIoKTtcbiAgfVxuICB2YXIgZmlyc3RPd25lckdyYXBoID0gZmlyc3ROb2RlLmdldE93bmVyKCk7XG5cbiAgZG9cbiAge1xuICAgIGlmIChmaXJzdE93bmVyR3JhcGggPT0gbnVsbClcbiAgICB7XG4gICAgICBicmVhaztcbiAgICB9XG4gICAgdmFyIHNlY29uZE93bmVyR3JhcGggPSBzZWNvbmROb2RlLmdldE93bmVyKCk7XG5cbiAgICBkb1xuICAgIHtcbiAgICAgIGlmIChzZWNvbmRPd25lckdyYXBoID09IG51bGwpXG4gICAgICB7XG4gICAgICAgIGJyZWFrO1xuICAgICAgfVxuXG4gICAgICBpZiAoc2Vjb25kT3duZXJHcmFwaCA9PSBmaXJzdE93bmVyR3JhcGgpXG4gICAgICB7XG4gICAgICAgIHJldHVybiBzZWNvbmRPd25lckdyYXBoO1xuICAgICAgfVxuICAgICAgc2Vjb25kT3duZXJHcmFwaCA9IHNlY29uZE93bmVyR3JhcGguZ2V0UGFyZW50KCkuZ2V0T3duZXIoKTtcbiAgICB9IHdoaWxlICh0cnVlKTtcblxuICAgIGZpcnN0T3duZXJHcmFwaCA9IGZpcnN0T3duZXJHcmFwaC5nZXRQYXJlbnQoKS5nZXRPd25lcigpO1xuICB9IHdoaWxlICh0cnVlKTtcblxuICByZXR1cm4gZmlyc3RPd25lckdyYXBoO1xufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuY2FsY0luY2x1c2lvblRyZWVEZXB0aHMgPSBmdW5jdGlvbiAoZ3JhcGgsIGRlcHRoKSB7XG4gIGlmIChncmFwaCA9PSBudWxsICYmIGRlcHRoID09IG51bGwpIHtcbiAgICBncmFwaCA9IHRoaXMucm9vdEdyYXBoO1xuICAgIGRlcHRoID0gMTtcbiAgfVxuICB2YXIgbm9kZTtcblxuICB2YXIgbm9kZXMgPSBncmFwaC5nZXROb2RlcygpO1xuICB2YXIgcyA9IG5vZGVzLmxlbmd0aDtcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBzOyBpKyspXG4gIHtcbiAgICBub2RlID0gbm9kZXNbaV07XG4gICAgbm9kZS5pbmNsdXNpb25UcmVlRGVwdGggPSBkZXB0aDtcblxuICAgIGlmIChub2RlLmNoaWxkICE9IG51bGwpXG4gICAge1xuICAgICAgdGhpcy5jYWxjSW5jbHVzaW9uVHJlZURlcHRocyhub2RlLmNoaWxkLCBkZXB0aCArIDEpO1xuICAgIH1cbiAgfVxufTtcblxuTEdyYXBoTWFuYWdlci5wcm90b3R5cGUuaW5jbHVkZXNJbnZhbGlkRWRnZSA9IGZ1bmN0aW9uICgpXG57XG4gIHZhciBlZGdlO1xuXG4gIHZhciBzID0gdGhpcy5lZGdlcy5sZW5ndGg7XG4gIGZvciAodmFyIGkgPSAwOyBpIDwgczsgaSsrKVxuICB7XG4gICAgZWRnZSA9IHRoaXMuZWRnZXNbaV07XG5cbiAgICBpZiAodGhpcy5pc09uZUFuY2VzdG9yT2ZPdGhlcihlZGdlLnNvdXJjZSwgZWRnZS50YXJnZXQpKVxuICAgIHtcbiAgICAgIHJldHVybiB0cnVlO1xuICAgIH1cbiAgfVxuICByZXR1cm4gZmFsc2U7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IExHcmFwaE1hbmFnZXI7XG4iLCJmdW5jdGlvbiBMR3JhcGhPYmplY3QodkdyYXBoT2JqZWN0KSB7XG4gIHRoaXMudkdyYXBoT2JqZWN0ID0gdkdyYXBoT2JqZWN0O1xufVxuXG5tb2R1bGUuZXhwb3J0cyA9IExHcmFwaE9iamVjdDtcbiIsInZhciBMR3JhcGhPYmplY3QgPSByZXF1aXJlKCcuL0xHcmFwaE9iamVjdCcpO1xudmFyIEludGVnZXIgPSByZXF1aXJlKCcuL0ludGVnZXInKTtcbnZhciBSZWN0YW5nbGVEID0gcmVxdWlyZSgnLi9SZWN0YW5nbGVEJyk7XG5cbmZ1bmN0aW9uIExOb2RlKGdtLCBsb2MsIHNpemUsIHZOb2RlKSB7XG4gIC8vQWx0ZXJuYXRpdmUgY29uc3RydWN0b3IgMSA6IExOb2RlKExHcmFwaE1hbmFnZXIgZ20sIFBvaW50IGxvYywgRGltZW5zaW9uIHNpemUsIE9iamVjdCB2Tm9kZSlcbiAgaWYgKHNpemUgPT0gbnVsbCAmJiB2Tm9kZSA9PSBudWxsKSB7XG4gICAgdk5vZGUgPSBsb2M7XG4gIH1cblxuICBMR3JhcGhPYmplY3QuY2FsbCh0aGlzLCB2Tm9kZSk7XG5cbiAgLy9BbHRlcm5hdGl2ZSBjb25zdHJ1Y3RvciAyIDogTE5vZGUoTGF5b3V0IGxheW91dCwgT2JqZWN0IHZOb2RlKVxuICBpZiAoZ20uZ3JhcGhNYW5hZ2VyICE9IG51bGwpXG4gICAgZ20gPSBnbS5ncmFwaE1hbmFnZXI7XG5cbiAgdGhpcy5lc3RpbWF0ZWRTaXplID0gSW50ZWdlci5NSU5fVkFMVUU7XG4gIHRoaXMuaW5jbHVzaW9uVHJlZURlcHRoID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gIHRoaXMudkdyYXBoT2JqZWN0ID0gdk5vZGU7XG4gIHRoaXMuZWRnZXMgPSBbXTtcbiAgdGhpcy5ncmFwaE1hbmFnZXIgPSBnbTtcblxuICBpZiAoc2l6ZSAhPSBudWxsICYmIGxvYyAhPSBudWxsKVxuICAgIHRoaXMucmVjdCA9IG5ldyBSZWN0YW5nbGVEKGxvYy54LCBsb2MueSwgc2l6ZS53aWR0aCwgc2l6ZS5oZWlnaHQpO1xuICBlbHNlXG4gICAgdGhpcy5yZWN0ID0gbmV3IFJlY3RhbmdsZUQoKTtcbn1cblxuTE5vZGUucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMR3JhcGhPYmplY3QucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gTEdyYXBoT2JqZWN0KSB7XG4gIExOb2RlW3Byb3BdID0gTEdyYXBoT2JqZWN0W3Byb3BdO1xufVxuXG5MTm9kZS5wcm90b3R5cGUuZ2V0RWRnZXMgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5lZGdlcztcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRDaGlsZCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmNoaWxkO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldE93bmVyID0gZnVuY3Rpb24gKClcbntcbiAgaWYgKHRoaXMub3duZXIgIT0gbnVsbCkge1xuICAgIGlmICghKHRoaXMub3duZXIgPT0gbnVsbCB8fCB0aGlzLm93bmVyLmdldE5vZGVzKCkuaW5kZXhPZih0aGlzKSA+IC0xKSkge1xuICAgICAgdGhyb3cgXCJhc3NlcnQgZmFpbGVkXCI7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIHRoaXMub3duZXI7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0V2lkdGggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LndpZHRoO1xufTtcblxuTE5vZGUucHJvdG90eXBlLnNldFdpZHRoID0gZnVuY3Rpb24gKHdpZHRoKVxue1xuICB0aGlzLnJlY3Qud2lkdGggPSB3aWR0aDtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRIZWlnaHQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LmhlaWdodDtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5zZXRIZWlnaHQgPSBmdW5jdGlvbiAoaGVpZ2h0KVxue1xuICB0aGlzLnJlY3QuaGVpZ2h0ID0gaGVpZ2h0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldENlbnRlclggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LnggKyB0aGlzLnJlY3Qud2lkdGggLyAyO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldENlbnRlclkgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LnkgKyB0aGlzLnJlY3QuaGVpZ2h0IC8gMjtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRDZW50ZXIgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLnJlY3QueCArIHRoaXMucmVjdC53aWR0aCAvIDIsXG4gICAgICAgICAgdGhpcy5yZWN0LnkgKyB0aGlzLnJlY3QuaGVpZ2h0IC8gMik7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0TG9jYXRpb24gPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLnJlY3QueCwgdGhpcy5yZWN0LnkpO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFJlY3QgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldERpYWdvbmFsID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIE1hdGguc3FydCh0aGlzLnJlY3Qud2lkdGggKiB0aGlzLnJlY3Qud2lkdGggK1xuICAgICAgICAgIHRoaXMucmVjdC5oZWlnaHQgKiB0aGlzLnJlY3QuaGVpZ2h0KTtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5zZXRSZWN0ID0gZnVuY3Rpb24gKHVwcGVyTGVmdCwgZGltZW5zaW9uKVxue1xuICB0aGlzLnJlY3QueCA9IHVwcGVyTGVmdC54O1xuICB0aGlzLnJlY3QueSA9IHVwcGVyTGVmdC55O1xuICB0aGlzLnJlY3Qud2lkdGggPSBkaW1lbnNpb24ud2lkdGg7XG4gIHRoaXMucmVjdC5oZWlnaHQgPSBkaW1lbnNpb24uaGVpZ2h0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLnNldENlbnRlciA9IGZ1bmN0aW9uIChjeCwgY3kpXG57XG4gIHRoaXMucmVjdC54ID0gY3ggLSB0aGlzLnJlY3Qud2lkdGggLyAyO1xuICB0aGlzLnJlY3QueSA9IGN5IC0gdGhpcy5yZWN0LmhlaWdodCAvIDI7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuc2V0TG9jYXRpb24gPSBmdW5jdGlvbiAoeCwgeSlcbntcbiAgdGhpcy5yZWN0LnggPSB4O1xuICB0aGlzLnJlY3QueSA9IHk7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUubW92ZUJ5ID0gZnVuY3Rpb24gKGR4LCBkeSlcbntcbiAgdGhpcy5yZWN0LnggKz0gZHg7XG4gIHRoaXMucmVjdC55ICs9IGR5O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldEVkZ2VMaXN0VG9Ob2RlID0gZnVuY3Rpb24gKHRvKVxue1xuICB2YXIgZWRnZUxpc3QgPSBbXTtcbiAgdmFyIGVkZ2U7XG5cbiAgZm9yICh2YXIgb2JqIGluIHRoaXMuZWRnZXMpXG4gIHtcbiAgICBlZGdlID0gb2JqO1xuXG4gICAgaWYgKGVkZ2UudGFyZ2V0ID09IHRvKVxuICAgIHtcbiAgICAgIGlmIChlZGdlLnNvdXJjZSAhPSB0aGlzKVxuICAgICAgICB0aHJvdyBcIkluY29ycmVjdCBlZGdlIHNvdXJjZSFcIjtcblxuICAgICAgZWRnZUxpc3QucHVzaChlZGdlKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gZWRnZUxpc3Q7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0RWRnZXNCZXR3ZWVuID0gZnVuY3Rpb24gKG90aGVyKVxue1xuICB2YXIgZWRnZUxpc3QgPSBbXTtcbiAgdmFyIGVkZ2U7XG5cbiAgZm9yICh2YXIgb2JqIGluIHRoaXMuZWRnZXMpXG4gIHtcbiAgICBlZGdlID0gdGhpcy5lZGdlc1tvYmpdO1xuXG4gICAgaWYgKCEoZWRnZS5zb3VyY2UgPT0gdGhpcyB8fCBlZGdlLnRhcmdldCA9PSB0aGlzKSlcbiAgICAgIHRocm93IFwiSW5jb3JyZWN0IGVkZ2Ugc291cmNlIGFuZC9vciB0YXJnZXRcIjtcblxuICAgIGlmICgoZWRnZS50YXJnZXQgPT0gb3RoZXIpIHx8IChlZGdlLnNvdXJjZSA9PSBvdGhlcikpXG4gICAge1xuICAgICAgZWRnZUxpc3QucHVzaChlZGdlKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gZWRnZUxpc3Q7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0TmVpZ2hib3JzTGlzdCA9IGZ1bmN0aW9uICgpXG57XG4gIHZhciBuZWlnaGJvcnMgPSBuZXcgSGFzaFNldCgpO1xuICB2YXIgZWRnZTtcblxuICBmb3IgKHZhciBvYmogaW4gdGhpcy5lZGdlcylcbiAge1xuICAgIGVkZ2UgPSB0aGlzLmVkZ2VzW29ial07XG5cbiAgICBpZiAoZWRnZS5zb3VyY2UgPT0gdGhpcylcbiAgICB7XG4gICAgICBuZWlnaGJvcnMuYWRkKGVkZ2UudGFyZ2V0KTtcbiAgICB9XG4gICAgZWxzZVxuICAgIHtcbiAgICAgIGlmICghZWRnZS50YXJnZXQgPT0gdGhpcylcbiAgICAgICAgdGhyb3cgXCJJbmNvcnJlY3QgaW5jaWRlbmN5IVwiO1xuICAgICAgbmVpZ2hib3JzLmFkZChlZGdlLnNvdXJjZSk7XG4gICAgfVxuICB9XG5cbiAgcmV0dXJuIG5laWdoYm9ycztcbn07XG5cbkxOb2RlLnByb3RvdHlwZS53aXRoQ2hpbGRyZW4gPSBmdW5jdGlvbiAoKVxue1xuICB2YXIgd2l0aE5laWdoYm9yc0xpc3QgPSBbXTtcbiAgdmFyIGNoaWxkTm9kZTtcblxuICB3aXRoTmVpZ2hib3JzTGlzdC5wdXNoKHRoaXMpO1xuXG4gIGlmICh0aGlzLmNoaWxkICE9IG51bGwpXG4gIHtcbiAgICB2YXIgbm9kZXMgPSB0aGlzLmNoaWxkLmdldE5vZGVzKCk7XG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKylcbiAgICB7XG4gICAgICBjaGlsZE5vZGUgPSBub2Rlc1tpXTtcblxuICAgICAgd2l0aE5laWdoYm9yc0xpc3QgPSB3aXRoTmVpZ2hib3JzTGlzdC5jb25jYXQoY2hpbGROb2RlLndpdGhDaGlsZHJlbigpKTtcbiAgICB9XG4gIH1cblxuICByZXR1cm4gd2l0aE5laWdoYm9yc0xpc3Q7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuZ2V0RXN0aW1hdGVkU2l6ZSA9IGZ1bmN0aW9uICgpIHtcbiAgaWYgKHRoaXMuZXN0aW1hdGVkU2l6ZSA9PSBJbnRlZ2VyLk1JTl9WQUxVRSkge1xuICAgIHRocm93IFwiYXNzZXJ0IGZhaWxlZFwiO1xuICB9XG4gIHJldHVybiB0aGlzLmVzdGltYXRlZFNpemU7XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuY2FsY0VzdGltYXRlZFNpemUgPSBmdW5jdGlvbiAoKSB7XG4gIGlmICh0aGlzLmNoaWxkID09IG51bGwpXG4gIHtcbiAgICByZXR1cm4gdGhpcy5lc3RpbWF0ZWRTaXplID0gTWF0aC5mbG9vcigodGhpcy5yZWN0LndpZHRoICsgdGhpcy5yZWN0LmhlaWdodCkgLyAyKTtcbiAgfVxuICBlbHNlXG4gIHtcbiAgICB0aGlzLmVzdGltYXRlZFNpemUgPSB0aGlzLmNoaWxkLmNhbGNFc3RpbWF0ZWRTaXplKCk7XG4gICAgdGhpcy5yZWN0LndpZHRoID0gdGhpcy5lc3RpbWF0ZWRTaXplO1xuICAgIHRoaXMucmVjdC5oZWlnaHQgPSB0aGlzLmVzdGltYXRlZFNpemU7XG5cbiAgICByZXR1cm4gdGhpcy5lc3RpbWF0ZWRTaXplO1xuICB9XG59O1xuXG5MTm9kZS5wcm90b3R5cGUuc2NhdHRlciA9IGZ1bmN0aW9uICgpIHtcbiAgdmFyIHJhbmRvbUNlbnRlclg7XG4gIHZhciByYW5kb21DZW50ZXJZO1xuXG4gIHZhciBtaW5YID0gLUxheW91dENvbnN0YW50cy5JTklUSUFMX1dPUkxEX0JPVU5EQVJZO1xuICB2YXIgbWF4WCA9IExheW91dENvbnN0YW50cy5JTklUSUFMX1dPUkxEX0JPVU5EQVJZO1xuICByYW5kb21DZW50ZXJYID0gTGF5b3V0Q29uc3RhbnRzLldPUkxEX0NFTlRFUl9YICtcbiAgICAgICAgICAoUmFuZG9tU2VlZC5uZXh0RG91YmxlKCkgKiAobWF4WCAtIG1pblgpKSArIG1pblg7XG5cbiAgdmFyIG1pblkgPSAtTGF5b3V0Q29uc3RhbnRzLklOSVRJQUxfV09STERfQk9VTkRBUlk7XG4gIHZhciBtYXhZID0gTGF5b3V0Q29uc3RhbnRzLklOSVRJQUxfV09STERfQk9VTkRBUlk7XG4gIHJhbmRvbUNlbnRlclkgPSBMYXlvdXRDb25zdGFudHMuV09STERfQ0VOVEVSX1kgK1xuICAgICAgICAgIChSYW5kb21TZWVkLm5leHREb3VibGUoKSAqIChtYXhZIC0gbWluWSkpICsgbWluWTtcblxuICB0aGlzLnJlY3QueCA9IHJhbmRvbUNlbnRlclg7XG4gIHRoaXMucmVjdC55ID0gcmFuZG9tQ2VudGVyWVxufTtcblxuTE5vZGUucHJvdG90eXBlLnVwZGF0ZUJvdW5kcyA9IGZ1bmN0aW9uICgpIHtcbiAgaWYgKHRoaXMuZ2V0Q2hpbGQoKSA9PSBudWxsKSB7XG4gICAgdGhyb3cgXCJhc3NlcnQgZmFpbGVkXCI7XG4gIH1cbiAgaWYgKHRoaXMuZ2V0Q2hpbGQoKS5nZXROb2RlcygpLmxlbmd0aCAhPSAwKVxuICB7XG4gICAgLy8gd3JhcCB0aGUgY2hpbGRyZW4gbm9kZXMgYnkgcmUtYXJyYW5naW5nIHRoZSBib3VuZGFyaWVzXG4gICAgdmFyIGNoaWxkR3JhcGggPSB0aGlzLmdldENoaWxkKCk7XG4gICAgY2hpbGRHcmFwaC51cGRhdGVCb3VuZHModHJ1ZSk7XG5cbiAgICB0aGlzLnJlY3QueCA9IGNoaWxkR3JhcGguZ2V0TGVmdCgpO1xuICAgIHRoaXMucmVjdC55ID0gY2hpbGRHcmFwaC5nZXRUb3AoKTtcblxuICAgIHRoaXMuc2V0V2lkdGgoY2hpbGRHcmFwaC5nZXRSaWdodCgpIC0gY2hpbGRHcmFwaC5nZXRMZWZ0KCkgK1xuICAgICAgICAgICAgMiAqIExheW91dENvbnN0YW50cy5DT01QT1VORF9OT0RFX01BUkdJTik7XG4gICAgdGhpcy5zZXRIZWlnaHQoY2hpbGRHcmFwaC5nZXRCb3R0b20oKSAtIGNoaWxkR3JhcGguZ2V0VG9wKCkgK1xuICAgICAgICAgICAgMiAqIExheW91dENvbnN0YW50cy5DT01QT1VORF9OT0RFX01BUkdJTiArXG4gICAgICAgICAgICBMYXlvdXRDb25zdGFudHMuTEFCRUxfSEVJR0hUKTtcbiAgfVxufTtcblxuTE5vZGUucHJvdG90eXBlLmdldEluY2x1c2lvblRyZWVEZXB0aCA9IGZ1bmN0aW9uICgpXG57XG4gIGlmICh0aGlzLmluY2x1c2lvblRyZWVEZXB0aCA9PSBJbnRlZ2VyLk1BWF9WQUxVRSkge1xuICAgIHRocm93IFwiYXNzZXJ0IGZhaWxlZFwiO1xuICB9XG4gIHJldHVybiB0aGlzLmluY2x1c2lvblRyZWVEZXB0aDtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS50cmFuc2Zvcm0gPSBmdW5jdGlvbiAodHJhbnMpXG57XG4gIHZhciBsZWZ0ID0gdGhpcy5yZWN0Lng7XG5cbiAgaWYgKGxlZnQgPiBMYXlvdXRDb25zdGFudHMuV09STERfQk9VTkRBUlkpXG4gIHtcbiAgICBsZWZ0ID0gTGF5b3V0Q29uc3RhbnRzLldPUkxEX0JPVU5EQVJZO1xuICB9XG4gIGVsc2UgaWYgKGxlZnQgPCAtTGF5b3V0Q29uc3RhbnRzLldPUkxEX0JPVU5EQVJZKVxuICB7XG4gICAgbGVmdCA9IC1MYXlvdXRDb25zdGFudHMuV09STERfQk9VTkRBUlk7XG4gIH1cblxuICB2YXIgdG9wID0gdGhpcy5yZWN0Lnk7XG5cbiAgaWYgKHRvcCA+IExheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWSlcbiAge1xuICAgIHRvcCA9IExheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWTtcbiAgfVxuICBlbHNlIGlmICh0b3AgPCAtTGF5b3V0Q29uc3RhbnRzLldPUkxEX0JPVU5EQVJZKVxuICB7XG4gICAgdG9wID0gLUxheW91dENvbnN0YW50cy5XT1JMRF9CT1VOREFSWTtcbiAgfVxuXG4gIHZhciBsZWZ0VG9wID0gbmV3IFBvaW50RChsZWZ0LCB0b3ApO1xuICB2YXIgdkxlZnRUb3AgPSB0cmFucy5pbnZlcnNlVHJhbnNmb3JtUG9pbnQobGVmdFRvcCk7XG5cbiAgdGhpcy5zZXRMb2NhdGlvbih2TGVmdFRvcC54LCB2TGVmdFRvcC55KTtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRMZWZ0ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMucmVjdC54O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFJpZ2h0ID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMucmVjdC54ICsgdGhpcy5yZWN0LndpZHRoO1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFRvcCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnJlY3QueTtcbn07XG5cbkxOb2RlLnByb3RvdHlwZS5nZXRCb3R0b20gPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5yZWN0LnkgKyB0aGlzLnJlY3QuaGVpZ2h0O1xufTtcblxuTE5vZGUucHJvdG90eXBlLmdldFBhcmVudCA9IGZ1bmN0aW9uICgpXG57XG4gIGlmICh0aGlzLm93bmVyID09IG51bGwpXG4gIHtcbiAgICByZXR1cm4gbnVsbDtcbiAgfVxuXG4gIHJldHVybiB0aGlzLm93bmVyLmdldFBhcmVudCgpO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSBMTm9kZTtcbiIsInZhciBMYXlvdXRDb25zdGFudHMgPSByZXF1aXJlKCcuL0xheW91dENvbnN0YW50cycpO1xyXG52YXIgSGFzaE1hcCA9IHJlcXVpcmUoJy4vSGFzaE1hcCcpO1xyXG52YXIgTEdyYXBoTWFuYWdlciA9IHJlcXVpcmUoJy4vTEdyYXBoTWFuYWdlcicpO1xyXG5cclxuZnVuY3Rpb24gTGF5b3V0KGlzUmVtb3RlVXNlKSB7XHJcbiAgLy9MYXlvdXQgUXVhbGl0eTogMDpwcm9vZiwgMTpkZWZhdWx0LCAyOmRyYWZ0XHJcbiAgdGhpcy5sYXlvdXRRdWFsaXR5ID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfUVVBTElUWTtcclxuICAvL1doZXRoZXIgbGF5b3V0IHNob3VsZCBjcmVhdGUgYmVuZHBvaW50cyBhcyBuZWVkZWQgb3Igbm90XHJcbiAgdGhpcy5jcmVhdGVCZW5kc0FzTmVlZGVkID1cclxuICAgICAgICAgIExheW91dENvbnN0YW50cy5ERUZBVUxUX0NSRUFURV9CRU5EU19BU19ORUVERUQ7XHJcbiAgLy9XaGV0aGVyIGxheW91dCBzaG91bGQgYmUgaW5jcmVtZW50YWwgb3Igbm90XHJcbiAgdGhpcy5pbmNyZW1lbnRhbCA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMO1xyXG4gIC8vV2hldGhlciB3ZSBhbmltYXRlIGZyb20gYmVmb3JlIHRvIGFmdGVyIGxheW91dCBub2RlIHBvc2l0aW9uc1xyXG4gIHRoaXMuYW5pbWF0aW9uT25MYXlvdXQgPVxyXG4gICAgICAgICAgTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX09OX0xBWU9VVDtcclxuICAvL1doZXRoZXIgd2UgYW5pbWF0ZSB0aGUgbGF5b3V0IHByb2Nlc3Mgb3Igbm90XHJcbiAgdGhpcy5hbmltYXRpb25EdXJpbmdMYXlvdXQgPSBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fRFVSSU5HX0xBWU9VVDtcclxuICAvL051bWJlciBpdGVyYXRpb25zIHRoYXQgc2hvdWxkIGJlIGRvbmUgYmV0d2VlbiB0d28gc3VjY2Vzc2l2ZSBhbmltYXRpb25zXHJcbiAgdGhpcy5hbmltYXRpb25QZXJpb2QgPSBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fUEVSSU9EO1xyXG4gIC8qKlxyXG4gICAqIFdoZXRoZXIgb3Igbm90IGxlYWYgbm9kZXMgKG5vbi1jb21wb3VuZCBub2RlcykgYXJlIG9mIHVuaWZvcm0gc2l6ZXMuIFdoZW5cclxuICAgKiB0aGV5IGFyZSwgYm90aCBzcHJpbmcgYW5kIHJlcHVsc2lvbiBmb3JjZXMgYmV0d2VlbiB0d28gbGVhZiBub2RlcyBjYW4gYmVcclxuICAgKiBjYWxjdWxhdGVkIHdpdGhvdXQgdGhlIGV4cGVuc2l2ZSBjbGlwcGluZyBwb2ludCBjYWxjdWxhdGlvbnMsIHJlc3VsdGluZ1xyXG4gICAqIGluIG1ham9yIHNwZWVkLXVwLlxyXG4gICAqL1xyXG4gIHRoaXMudW5pZm9ybUxlYWZOb2RlU2l6ZXMgPVxyXG4gICAgICAgICAgTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfVU5JRk9STV9MRUFGX05PREVfU0laRVM7XHJcbiAgLyoqXHJcbiAgICogVGhpcyBpcyB1c2VkIGZvciBjcmVhdGlvbiBvZiBiZW5kcG9pbnRzIGJ5IHVzaW5nIGR1bW15IG5vZGVzIGFuZCBlZGdlcy5cclxuICAgKiBNYXBzIGFuIExFZGdlIHRvIGl0cyBkdW1teSBiZW5kcG9pbnQgcGF0aC5cclxuICAgKi9cclxuICB0aGlzLmVkZ2VUb0R1bW15Tm9kZXMgPSBuZXcgSGFzaE1hcCgpO1xyXG4gIHRoaXMuZ3JhcGhNYW5hZ2VyID0gbmV3IExHcmFwaE1hbmFnZXIodGhpcyk7XHJcbiAgdGhpcy5pc0xheW91dEZpbmlzaGVkID0gZmFsc2U7XHJcbiAgdGhpcy5pc1N1YkxheW91dCA9IGZhbHNlO1xyXG4gIHRoaXMuaXNSZW1vdGVVc2UgPSBmYWxzZTtcclxuXHJcbiAgaWYgKGlzUmVtb3RlVXNlICE9IG51bGwpIHtcclxuICAgIHRoaXMuaXNSZW1vdGVVc2UgPSBpc1JlbW90ZVVzZTtcclxuICB9XHJcbn1cclxuXHJcbkxheW91dC5SQU5ET01fU0VFRCA9IDE7XHJcblxyXG5MYXlvdXQucHJvdG90eXBlLmdldEdyYXBoTWFuYWdlciA9IGZ1bmN0aW9uICgpIHtcclxuICByZXR1cm4gdGhpcy5ncmFwaE1hbmFnZXI7XHJcbn07XHJcblxyXG5MYXlvdXQucHJvdG90eXBlLmdldEFsbE5vZGVzID0gZnVuY3Rpb24gKCkge1xyXG4gIHJldHVybiB0aGlzLmdyYXBoTWFuYWdlci5nZXRBbGxOb2RlcygpO1xyXG59O1xyXG5cclxuTGF5b3V0LnByb3RvdHlwZS5nZXRBbGxFZGdlcyA9IGZ1bmN0aW9uICgpIHtcclxuICByZXR1cm4gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsRWRnZXMoKTtcclxufTtcclxuXHJcbkxheW91dC5wcm90b3R5cGUuZ2V0QWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24gPSBmdW5jdGlvbiAoKSB7XHJcbiAgcmV0dXJuIHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEFsbE5vZGVzVG9BcHBseUdyYXZpdGF0aW9uKCk7XHJcbn07XHJcblxyXG5MYXlvdXQucHJvdG90eXBlLm5ld0dyYXBoTWFuYWdlciA9IGZ1bmN0aW9uICgpIHtcclxuICB2YXIgZ20gPSBuZXcgTEdyYXBoTWFuYWdlcih0aGlzKTtcclxuICB0aGlzLmdyYXBoTWFuYWdlciA9IGdtO1xyXG4gIHJldHVybiBnbTtcclxufTtcclxuXHJcbkxheW91dC5wcm90b3R5cGUubmV3R3JhcGggPSBmdW5jdGlvbiAodkdyYXBoKVxyXG57XHJcbiAgcmV0dXJuIG5ldyBMR3JhcGgobnVsbCwgdGhpcy5ncmFwaE1hbmFnZXIsIHZHcmFwaCk7XHJcbn07XHJcblxyXG5MYXlvdXQucHJvdG90eXBlLm5ld05vZGUgPSBmdW5jdGlvbiAodk5vZGUpXHJcbntcclxuICByZXR1cm4gbmV3IExOb2RlKHRoaXMuZ3JhcGhNYW5hZ2VyLCB2Tm9kZSk7XHJcbn07XHJcblxyXG5MYXlvdXQucHJvdG90eXBlLm5ld0VkZ2UgPSBmdW5jdGlvbiAodkVkZ2UpXHJcbntcclxuICByZXR1cm4gbmV3IExFZGdlKG51bGwsIG51bGwsIHZFZGdlKTtcclxufTtcclxuXHJcbkxheW91dC5wcm90b3R5cGUucnVuTGF5b3V0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gIHRoaXMuaXNMYXlvdXRGaW5pc2hlZCA9IGZhbHNlO1xyXG5cclxuICB0aGlzLmluaXRQYXJhbWV0ZXJzKCk7XHJcbiAgdmFyIGlzTGF5b3V0U3VjY2Vzc2Z1bGw7XHJcblxyXG4gIGlmICgodGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpID09IG51bGwpXHJcbiAgICAgICAgICB8fCB0aGlzLmdyYXBoTWFuYWdlci5nZXRSb290KCkuZ2V0Tm9kZXMoKS5sZW5ndGggPT0gMFxyXG4gICAgICAgICAgfHwgdGhpcy5ncmFwaE1hbmFnZXIuaW5jbHVkZXNJbnZhbGlkRWRnZSgpKVxyXG4gIHtcclxuICAgIGlzTGF5b3V0U3VjY2Vzc2Z1bGwgPSBmYWxzZTtcclxuICB9XHJcbiAgZWxzZVxyXG4gIHtcclxuICAgIC8vIGNhbGN1bGF0ZSBleGVjdXRpb24gdGltZVxyXG4gICAgdmFyIHN0YXJ0VGltZSA9IDA7XHJcblxyXG4gICAgaWYgKCF0aGlzLmlzU3ViTGF5b3V0KVxyXG4gICAge1xyXG4gICAgICBzdGFydFRpbWUgPSBuZXcgRGF0ZSgpLmdldFRpbWUoKVxyXG4gICAgfVxyXG5cclxuICAgIGlzTGF5b3V0U3VjY2Vzc2Z1bGwgPSB0aGlzLmxheW91dCgpO1xyXG5cclxuICAgIGlmICghdGhpcy5pc1N1YkxheW91dClcclxuICAgIHtcclxuICAgICAgdmFyIGVuZFRpbWUgPSBuZXcgRGF0ZSgpLmdldFRpbWUoKTtcclxuICAgICAgdmFyIGV4Y1RpbWUgPSBlbmRUaW1lIC0gc3RhcnRUaW1lO1xyXG5cclxuICAgICAgY29uc29sZS5sb2coXCJUb3RhbCBleGVjdXRpb24gdGltZTogXCIgKyBleGNUaW1lICsgXCIgbWlsaXNlY29uZHMuXCIpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgaWYgKGlzTGF5b3V0U3VjY2Vzc2Z1bGwpXHJcbiAge1xyXG4gICAgaWYgKCF0aGlzLmlzU3ViTGF5b3V0KVxyXG4gICAge1xyXG4gICAgICB0aGlzLmRvUG9zdExheW91dCgpO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgdGhpcy5pc0xheW91dEZpbmlzaGVkID0gdHJ1ZTtcclxuXHJcbiAgcmV0dXJuIGlzTGF5b3V0U3VjY2Vzc2Z1bGw7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgcGVyZm9ybXMgdGhlIG9wZXJhdGlvbnMgcmVxdWlyZWQgYWZ0ZXIgbGF5b3V0LlxyXG4gKi9cclxuTGF5b3V0LnByb3RvdHlwZS5kb1Bvc3RMYXlvdXQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgLy9hc3NlcnQgIWlzU3ViTGF5b3V0IDogXCJTaG91bGQgbm90IGJlIGNhbGxlZCBvbiBzdWItbGF5b3V0IVwiO1xyXG4gIC8vIFByb3BhZ2F0ZSBnZW9tZXRyaWMgY2hhbmdlcyB0byB2LWxldmVsIG9iamVjdHNcclxuICB0aGlzLnRyYW5zZm9ybSgpO1xyXG4gIHRoaXMudXBkYXRlKCk7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgdXBkYXRlcyB0aGUgZ2VvbWV0cnkgb2YgdGhlIHRhcmdldCBncmFwaCBhY2NvcmRpbmcgdG9cclxuICogY2FsY3VsYXRlZCBsYXlvdXQuXHJcbiAqL1xyXG5MYXlvdXQucHJvdG90eXBlLnVwZGF0ZTIgPSBmdW5jdGlvbiAoKSB7XHJcbiAgLy8gdXBkYXRlIGJlbmQgcG9pbnRzXHJcbiAgaWYgKHRoaXMuY3JlYXRlQmVuZHNBc05lZWRlZClcclxuICB7XHJcbiAgICB0aGlzLmNyZWF0ZUJlbmRwb2ludHNGcm9tRHVtbXlOb2RlcygpO1xyXG5cclxuICAgIC8vIHJlc2V0IGFsbCBlZGdlcywgc2luY2UgdGhlIHRvcG9sb2d5IGhhcyBjaGFuZ2VkXHJcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5yZXNldEFsbEVkZ2VzKCk7XHJcbiAgfVxyXG5cclxuICAvLyBwZXJmb3JtIGVkZ2UsIG5vZGUgYW5kIHJvb3QgdXBkYXRlcyBpZiBsYXlvdXQgaXMgbm90IGNhbGxlZFxyXG4gIC8vIHJlbW90ZWx5XHJcbiAgaWYgKCF0aGlzLmlzUmVtb3RlVXNlKVxyXG4gIHtcclxuICAgIC8vIHVwZGF0ZSBhbGwgZWRnZXNcclxuICAgIHZhciBlZGdlO1xyXG4gICAgdmFyIGFsbEVkZ2VzID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsRWRnZXMoKTtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgYWxsRWRnZXMubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgIGVkZ2UgPSBhbGxFZGdlc1tpXTtcclxuLy8gICAgICB0aGlzLnVwZGF0ZShlZGdlKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyByZWN1cnNpdmVseSB1cGRhdGUgbm9kZXNcclxuICAgIHZhciBub2RlO1xyXG4gICAgdmFyIG5vZGVzID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldE5vZGVzKCk7XHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IG5vZGVzLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICBub2RlID0gbm9kZXNbaV07XHJcbi8vICAgICAgdGhpcy51cGRhdGUobm9kZSk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gdXBkYXRlIHJvb3QgZ3JhcGhcclxuICAgIHRoaXMudXBkYXRlKHRoaXMuZ3JhcGhNYW5hZ2VyLmdldFJvb3QoKSk7XHJcbiAgfVxyXG59O1xyXG5cclxuTGF5b3V0LnByb3RvdHlwZS51cGRhdGUgPSBmdW5jdGlvbiAob2JqKSB7XHJcbiAgaWYgKG9iaiA9PSBudWxsKSB7XHJcbiAgICB0aGlzLnVwZGF0ZTIoKTtcclxuICB9XHJcbiAgZWxzZSBpZiAob2JqIGluc3RhbmNlb2YgTE5vZGUpIHtcclxuICAgIHZhciBub2RlID0gb2JqO1xyXG4gICAgaWYgKG5vZGUuZ2V0Q2hpbGQoKSAhPSBudWxsKVxyXG4gICAge1xyXG4gICAgICAvLyBzaW5jZSBub2RlIGlzIGNvbXBvdW5kLCByZWN1cnNpdmVseSB1cGRhdGUgY2hpbGQgbm9kZXNcclxuICAgICAgdmFyIG5vZGVzID0gbm9kZS5nZXRDaGlsZCgpLmdldE5vZGVzKCk7XHJcbiAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbm9kZXMubGVuZ3RoOyBpKyspXHJcbiAgICAgIHtcclxuICAgICAgICB1cGRhdGUobm9kZXNbaV0pO1xyXG4gICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gaWYgdGhlIGwtbGV2ZWwgbm9kZSBpcyBhc3NvY2lhdGVkIHdpdGggYSB2LWxldmVsIGdyYXBoIG9iamVjdCxcclxuICAgIC8vIHRoZW4gaXQgaXMgYXNzdW1lZCB0aGF0IHRoZSB2LWxldmVsIG5vZGUgaW1wbGVtZW50cyB0aGVcclxuICAgIC8vIGludGVyZmFjZSBVcGRhdGFibGUuXHJcbiAgICBpZiAobm9kZS52R3JhcGhPYmplY3QgIT0gbnVsbClcclxuICAgIHtcclxuICAgICAgLy8gY2FzdCB0byBVcGRhdGFibGUgd2l0aG91dCBhbnkgdHlwZSBjaGVja1xyXG4gICAgICB2YXIgdk5vZGUgPSBub2RlLnZHcmFwaE9iamVjdDtcclxuXHJcbiAgICAgIC8vIGNhbGwgdGhlIHVwZGF0ZSBtZXRob2Qgb2YgdGhlIGludGVyZmFjZVxyXG4gICAgICB2Tm9kZS51cGRhdGUobm9kZSk7XHJcbiAgICB9XHJcbiAgfVxyXG4gIGVsc2UgaWYgKG9iaiBpbnN0YW5jZW9mIExFZGdlKSB7XHJcbiAgICB2YXIgZWRnZSA9IG9iajtcclxuICAgIC8vIGlmIHRoZSBsLWxldmVsIGVkZ2UgaXMgYXNzb2NpYXRlZCB3aXRoIGEgdi1sZXZlbCBncmFwaCBvYmplY3QsXHJcbiAgICAvLyB0aGVuIGl0IGlzIGFzc3VtZWQgdGhhdCB0aGUgdi1sZXZlbCBlZGdlIGltcGxlbWVudHMgdGhlXHJcbiAgICAvLyBpbnRlcmZhY2UgVXBkYXRhYmxlLlxyXG5cclxuICAgIGlmIChlZGdlLnZHcmFwaE9iamVjdCAhPSBudWxsKVxyXG4gICAge1xyXG4gICAgICAvLyBjYXN0IHRvIFVwZGF0YWJsZSB3aXRob3V0IGFueSB0eXBlIGNoZWNrXHJcbiAgICAgIHZhciB2RWRnZSA9IGVkZ2UudkdyYXBoT2JqZWN0O1xyXG5cclxuICAgICAgLy8gY2FsbCB0aGUgdXBkYXRlIG1ldGhvZCBvZiB0aGUgaW50ZXJmYWNlXHJcbiAgICAgIHZFZGdlLnVwZGF0ZShlZGdlKTtcclxuICAgIH1cclxuICB9XHJcbiAgZWxzZSBpZiAob2JqIGluc3RhbmNlb2YgTEdyYXBoKSB7XHJcbiAgICB2YXIgZ3JhcGggPSBvYmo7XHJcbiAgICAvLyBpZiB0aGUgbC1sZXZlbCBncmFwaCBpcyBhc3NvY2lhdGVkIHdpdGggYSB2LWxldmVsIGdyYXBoIG9iamVjdCxcclxuICAgIC8vIHRoZW4gaXQgaXMgYXNzdW1lZCB0aGF0IHRoZSB2LWxldmVsIG9iamVjdCBpbXBsZW1lbnRzIHRoZVxyXG4gICAgLy8gaW50ZXJmYWNlIFVwZGF0YWJsZS5cclxuXHJcbiAgICBpZiAoZ3JhcGgudkdyYXBoT2JqZWN0ICE9IG51bGwpXHJcbiAgICB7XHJcbiAgICAgIC8vIGNhc3QgdG8gVXBkYXRhYmxlIHdpdGhvdXQgYW55IHR5cGUgY2hlY2tcclxuICAgICAgdmFyIHZHcmFwaCA9IGdyYXBoLnZHcmFwaE9iamVjdDtcclxuXHJcbiAgICAgIC8vIGNhbGwgdGhlIHVwZGF0ZSBtZXRob2Qgb2YgdGhlIGludGVyZmFjZVxyXG4gICAgICB2R3JhcGgudXBkYXRlKGdyYXBoKTtcclxuICAgIH1cclxuICB9XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgaXMgdXNlZCB0byBzZXQgYWxsIGxheW91dCBwYXJhbWV0ZXJzIHRvIGRlZmF1bHQgdmFsdWVzXHJcbiAqIGRldGVybWluZWQgYXQgY29tcGlsZSB0aW1lLlxyXG4gKi9cclxuTGF5b3V0LnByb3RvdHlwZS5pbml0UGFyYW1ldGVycyA9IGZ1bmN0aW9uICgpIHtcclxuICBpZiAoIXRoaXMuaXNTdWJMYXlvdXQpXHJcbiAge1xyXG4gICAgdGhpcy5sYXlvdXRRdWFsaXR5ID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfUVVBTElUWTtcclxuICAgIHRoaXMuYW5pbWF0aW9uRHVyaW5nTGF5b3V0ID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX09OX0xBWU9VVDtcclxuICAgIHRoaXMuYW5pbWF0aW9uUGVyaW9kID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX1BFUklPRDtcclxuICAgIHRoaXMuYW5pbWF0aW9uT25MYXlvdXQgPSBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fRFVSSU5HX0xBWU9VVDtcclxuICAgIHRoaXMuaW5jcmVtZW50YWwgPSBMYXlvdXRDb25zdGFudHMuREVGQVVMVF9JTkNSRU1FTlRBTDtcclxuICAgIHRoaXMuY3JlYXRlQmVuZHNBc05lZWRlZCA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX0NSRUFURV9CRU5EU19BU19ORUVERUQ7XHJcbiAgICB0aGlzLnVuaWZvcm1MZWFmTm9kZVNpemVzID0gTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfVU5JRk9STV9MRUFGX05PREVfU0laRVM7XHJcbiAgfVxyXG5cclxuICBpZiAodGhpcy5hbmltYXRpb25EdXJpbmdMYXlvdXQpXHJcbiAge1xyXG4gICAgYW5pbWF0aW9uT25MYXlvdXQgPSBmYWxzZTtcclxuICB9XHJcbn07XHJcblxyXG5MYXlvdXQucHJvdG90eXBlLnRyYW5zZm9ybSA9IGZ1bmN0aW9uIChuZXdMZWZ0VG9wKSB7XHJcbiAgaWYgKG5ld0xlZnRUb3AgPT0gdW5kZWZpbmVkKSB7XHJcbiAgICB0aGlzLnRyYW5zZm9ybShuZXcgUG9pbnREKDAsIDApKTtcclxuICB9XHJcbiAgZWxzZSB7XHJcbiAgICAvLyBjcmVhdGUgYSB0cmFuc2Zvcm1hdGlvbiBvYmplY3QgKGZyb20gRWNsaXBzZSB0byBsYXlvdXQpLiBXaGVuIGFuXHJcbiAgICAvLyBpbnZlcnNlIHRyYW5zZm9ybSBpcyBhcHBsaWVkLCB3ZSBnZXQgdXBwZXItbGVmdCBjb29yZGluYXRlIG9mIHRoZVxyXG4gICAgLy8gZHJhd2luZyBvciB0aGUgcm9vdCBncmFwaCBhdCBnaXZlbiBpbnB1dCBjb29yZGluYXRlIChzb21lIG1hcmdpbnNcclxuICAgIC8vIGFscmVhZHkgaW5jbHVkZWQgaW4gY2FsY3VsYXRpb24gb2YgbGVmdC10b3ApLlxyXG5cclxuICAgIHZhciB0cmFucyA9IG5ldyBUcmFuc2Zvcm0oKTtcclxuICAgIHZhciBsZWZ0VG9wID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLnVwZGF0ZUxlZnRUb3AoKTtcclxuXHJcbiAgICBpZiAobGVmdFRvcCAhPSBudWxsKVxyXG4gICAge1xyXG4gICAgICB0cmFucy5zZXRXb3JsZE9yZ1gobmV3TGVmdFRvcC54KTtcclxuICAgICAgdHJhbnMuc2V0V29ybGRPcmdZKG5ld0xlZnRUb3AueSk7XHJcblxyXG4gICAgICB0cmFucy5zZXREZXZpY2VPcmdYKGxlZnRUb3AueCk7XHJcbiAgICAgIHRyYW5zLnNldERldmljZU9yZ1kobGVmdFRvcC55KTtcclxuXHJcbiAgICAgIHZhciBub2RlcyA9IHRoaXMuZ2V0QWxsTm9kZXMoKTtcclxuICAgICAgdmFyIG5vZGU7XHJcblxyXG4gICAgICBmb3IgKHZhciBpID0gMDsgaSA8IG5vZGVzLmxlbmd0aDsgaSsrKVxyXG4gICAgICB7XHJcbiAgICAgICAgbm9kZSA9IG5vZGVzW2ldO1xyXG4gICAgICAgIG5vZGUudHJhbnNmb3JtKHRyYW5zKTtcclxuICAgICAgfVxyXG4gICAgfVxyXG4gIH1cclxufTtcclxuXHJcbkxheW91dC5wcm90b3R5cGUucG9zaXRpb25Ob2Rlc1JhbmRvbWx5ID0gZnVuY3Rpb24gKGdyYXBoKSB7XHJcblxyXG4gIGlmIChncmFwaCA9PSB1bmRlZmluZWQpIHtcclxuICAgIC8vYXNzZXJ0ICF0aGlzLmluY3JlbWVudGFsO1xyXG4gICAgdGhpcy5wb3NpdGlvbk5vZGVzUmFuZG9tbHkodGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRSb290KCkpO1xyXG4gICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRSb290KCkudXBkYXRlQm91bmRzKHRydWUpO1xyXG4gIH1cclxuICBlbHNlIHtcclxuICAgIHZhciBsTm9kZTtcclxuICAgIHZhciBjaGlsZEdyYXBoO1xyXG5cclxuICAgIHZhciBub2RlcyA9IGdyYXBoLmdldE5vZGVzKCk7XHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IG5vZGVzLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICBsTm9kZSA9IG5vZGVzW2ldO1xyXG4gICAgICBjaGlsZEdyYXBoID0gbE5vZGUuZ2V0Q2hpbGQoKTtcclxuXHJcbiAgICAgIGlmIChjaGlsZEdyYXBoID09IG51bGwpXHJcbiAgICAgIHtcclxuICAgICAgICBsTm9kZS5zY2F0dGVyKCk7XHJcbiAgICAgIH1cclxuICAgICAgZWxzZSBpZiAoY2hpbGRHcmFwaC5nZXROb2RlcygpLmxlbmd0aCA9PSAwKVxyXG4gICAgICB7XHJcbiAgICAgICAgbE5vZGUuc2NhdHRlcigpO1xyXG4gICAgICB9XHJcbiAgICAgIGVsc2VcclxuICAgICAge1xyXG4gICAgICAgIHRoaXMucG9zaXRpb25Ob2Rlc1JhbmRvbWx5KGNoaWxkR3JhcGgpO1xyXG4gICAgICAgIGxOb2RlLnVwZGF0ZUJvdW5kcygpO1xyXG4gICAgICB9XHJcbiAgICB9XHJcbiAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiAqIFRoaXMgbWV0aG9kIHJldHVybnMgYSBsaXN0IG9mIHRyZWVzIHdoZXJlIGVhY2ggdHJlZSBpcyByZXByZXNlbnRlZCBhcyBhXHJcbiAqIGxpc3Qgb2YgbC1ub2Rlcy4gVGhlIG1ldGhvZCByZXR1cm5zIGEgbGlzdCBvZiBzaXplIDAgd2hlbjpcclxuICogLSBUaGUgZ3JhcGggaXMgbm90IGZsYXQgb3JcclxuICogLSBPbmUgb2YgdGhlIGNvbXBvbmVudChzKSBvZiB0aGUgZ3JhcGggaXMgbm90IGEgdHJlZS5cclxuICovXHJcbkxheW91dC5wcm90b3R5cGUuZ2V0RmxhdEZvcmVzdCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICB2YXIgZmxhdEZvcmVzdCA9IFtdO1xyXG4gIHZhciBpc0ZvcmVzdCA9IHRydWU7XHJcblxyXG4gIC8vIFF1aWNrIHJlZmVyZW5jZSBmb3IgYWxsIG5vZGVzIGluIHRoZSBncmFwaCBtYW5hZ2VyIGFzc29jaWF0ZWQgd2l0aFxyXG4gIC8vIHRoaXMgbGF5b3V0LiBUaGUgbGlzdCBzaG91bGQgbm90IGJlIGNoYW5nZWQuXHJcbiAgdmFyIGFsbE5vZGVzID0gdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldE5vZGVzKCk7XHJcblxyXG4gIC8vIEZpcnN0IGJlIHN1cmUgdGhhdCB0aGUgZ3JhcGggaXMgZmxhdFxyXG4gIHZhciBpc0ZsYXQgPSB0cnVlO1xyXG5cclxuICBmb3IgKHZhciBpID0gMDsgaSA8IGFsbE5vZGVzLmxlbmd0aDsgaSsrKVxyXG4gIHtcclxuICAgIGlmIChhbGxOb2Rlc1tpXS5nZXRDaGlsZCgpICE9IG51bGwpXHJcbiAgICB7XHJcbiAgICAgIGlzRmxhdCA9IGZhbHNlO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgLy8gUmV0dXJuIGVtcHR5IGZvcmVzdCBpZiB0aGUgZ3JhcGggaXMgbm90IGZsYXQuXHJcbiAgaWYgKCFpc0ZsYXQpXHJcbiAge1xyXG4gICAgcmV0dXJuIGZsYXRGb3Jlc3Q7XHJcbiAgfVxyXG5cclxuICAvLyBSdW4gQkZTIGZvciBlYWNoIGNvbXBvbmVudCBvZiB0aGUgZ3JhcGguXHJcblxyXG4gIHZhciB2aXNpdGVkID0gbmV3IEhhc2hTZXQoKTtcclxuICB2YXIgdG9CZVZpc2l0ZWQgPSBbXTtcclxuICB2YXIgcGFyZW50cyA9IG5ldyBIYXNoTWFwKCk7XHJcbiAgdmFyIHVuUHJvY2Vzc2VkTm9kZXMgPSBbXTtcclxuXHJcbiAgdW5Qcm9jZXNzZWROb2RlcyA9IHVuUHJvY2Vzc2VkTm9kZXMuY29uY2F0KGFsbE5vZGVzKTtcclxuXHJcbiAgLy8gRWFjaCBpdGVyYXRpb24gb2YgdGhpcyBsb29wIGZpbmRzIGEgY29tcG9uZW50IG9mIHRoZSBncmFwaCBhbmRcclxuICAvLyBkZWNpZGVzIHdoZXRoZXIgaXQgaXMgYSB0cmVlIG9yIG5vdC4gSWYgaXQgaXMgYSB0cmVlLCBhZGRzIGl0IHRvIHRoZVxyXG4gIC8vIGZvcmVzdCBhbmQgY29udGludWVkIHdpdGggdGhlIG5leHQgY29tcG9uZW50LlxyXG5cclxuICB3aGlsZSAodW5Qcm9jZXNzZWROb2Rlcy5sZW5ndGggPiAwICYmIGlzRm9yZXN0KVxyXG4gIHtcclxuICAgIHRvQmVWaXNpdGVkLnB1c2godW5Qcm9jZXNzZWROb2Rlc1swXSk7XHJcblxyXG4gICAgLy8gU3RhcnQgdGhlIEJGUy4gRWFjaCBpdGVyYXRpb24gb2YgdGhpcyBsb29wIHZpc2l0cyBhIG5vZGUgaW4gYVxyXG4gICAgLy8gQkZTIG1hbm5lci5cclxuICAgIHdoaWxlICh0b0JlVmlzaXRlZC5sZW5ndGggPiAwICYmIGlzRm9yZXN0KVxyXG4gICAge1xyXG4gICAgICAvL3Bvb2wgb3BlcmF0aW9uXHJcbiAgICAgIHZhciBjdXJyZW50Tm9kZSA9IHRvQmVWaXNpdGVkWzBdO1xyXG4gICAgICB0b0JlVmlzaXRlZC5zcGxpY2UoMCwgMSk7XHJcbiAgICAgIHZpc2l0ZWQuYWRkKGN1cnJlbnROb2RlKTtcclxuXHJcbiAgICAgIC8vIFRyYXZlcnNlIGFsbCBuZWlnaGJvcnMgb2YgdGhpcyBub2RlXHJcbiAgICAgIHZhciBuZWlnaGJvckVkZ2VzID0gY3VycmVudE5vZGUuZ2V0RWRnZXMoKTtcclxuXHJcbiAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbmVpZ2hib3JFZGdlcy5sZW5ndGg7IGkrKylcclxuICAgICAge1xyXG4gICAgICAgIHZhciBjdXJyZW50TmVpZ2hib3IgPVxyXG4gICAgICAgICAgICAgICAgbmVpZ2hib3JFZGdlc1tpXS5nZXRPdGhlckVuZChjdXJyZW50Tm9kZSk7XHJcblxyXG4gICAgICAgIC8vIElmIEJGUyBpcyBub3QgZ3Jvd2luZyBmcm9tIHRoaXMgbmVpZ2hib3IuXHJcbiAgICAgICAgaWYgKHBhcmVudHMuZ2V0KGN1cnJlbnROb2RlKSAhPSBjdXJyZW50TmVpZ2hib3IpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgLy8gV2UgaGF2ZW4ndCBwcmV2aW91c2x5IHZpc2l0ZWQgdGhpcyBuZWlnaGJvci5cclxuICAgICAgICAgIGlmICghdmlzaXRlZC5jb250YWlucyhjdXJyZW50TmVpZ2hib3IpKVxyXG4gICAgICAgICAge1xyXG4gICAgICAgICAgICB0b0JlVmlzaXRlZC5wdXNoKGN1cnJlbnROZWlnaGJvcik7XHJcbiAgICAgICAgICAgIHBhcmVudHMucHV0KGN1cnJlbnROZWlnaGJvciwgY3VycmVudE5vZGUpO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgICAgLy8gU2luY2Ugd2UgaGF2ZSBwcmV2aW91c2x5IHZpc2l0ZWQgdGhpcyBuZWlnaGJvciBhbmRcclxuICAgICAgICAgIC8vIHRoaXMgbmVpZ2hib3IgaXMgbm90IHBhcmVudCBvZiBjdXJyZW50Tm9kZSwgZ2l2ZW5cclxuICAgICAgICAgIC8vIGdyYXBoIGNvbnRhaW5zIGEgY29tcG9uZW50IHRoYXQgaXMgbm90IHRyZWUsIGhlbmNlXHJcbiAgICAgICAgICAvLyBpdCBpcyBub3QgYSBmb3Jlc3QuXHJcbiAgICAgICAgICBlbHNlXHJcbiAgICAgICAgICB7XHJcbiAgICAgICAgICAgIGlzRm9yZXN0ID0gZmFsc2U7XHJcbiAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8vIFRoZSBncmFwaCBjb250YWlucyBhIGNvbXBvbmVudCB0aGF0IGlzIG5vdCBhIHRyZWUuIEVtcHR5XHJcbiAgICAvLyBwcmV2aW91c2x5IGZvdW5kIHRyZWVzLiBUaGUgbWV0aG9kIHdpbGwgZW5kLlxyXG4gICAgaWYgKCFpc0ZvcmVzdClcclxuICAgIHtcclxuICAgICAgZmxhdEZvcmVzdCA9IFtdO1xyXG4gICAgfVxyXG4gICAgLy8gU2F2ZSBjdXJyZW50bHkgdmlzaXRlZCBub2RlcyBhcyBhIHRyZWUgaW4gb3VyIGZvcmVzdC4gUmVzZXRcclxuICAgIC8vIHZpc2l0ZWQgYW5kIHBhcmVudHMgbGlzdHMuIENvbnRpbnVlIHdpdGggdGhlIG5leHQgY29tcG9uZW50IG9mXHJcbiAgICAvLyB0aGUgZ3JhcGgsIGlmIGFueS5cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgdmFyIHRlbXAgPSBbXTtcclxuICAgICAgdmlzaXRlZC5hZGRBbGxUbyh0ZW1wKTtcclxuICAgICAgZmxhdEZvcmVzdC5wdXNoKHRlbXApO1xyXG4gICAgICAvL2ZsYXRGb3Jlc3QgPSBmbGF0Rm9yZXN0LmNvbmNhdCh0ZW1wKTtcclxuICAgICAgLy91blByb2Nlc3NlZE5vZGVzLnJlbW92ZUFsbCh2aXNpdGVkKTtcclxuICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCB0ZW1wLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgdmFyIHZhbHVlID0gdGVtcFtpXTtcclxuICAgICAgICB2YXIgaW5kZXggPSB1blByb2Nlc3NlZE5vZGVzLmluZGV4T2YodmFsdWUpO1xyXG4gICAgICAgIGlmIChpbmRleCA+IC0xKSB7XHJcbiAgICAgICAgICB1blByb2Nlc3NlZE5vZGVzLnNwbGljZShpbmRleCwgMSk7XHJcbiAgICAgICAgfVxyXG4gICAgICB9XHJcbiAgICAgIHZpc2l0ZWQgPSBuZXcgSGFzaFNldCgpO1xyXG4gICAgICBwYXJlbnRzID0gbmV3IEhhc2hNYXAoKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHJldHVybiBmbGF0Rm9yZXN0O1xyXG59O1xyXG5cclxuLyoqXHJcbiAqIFRoaXMgbWV0aG9kIGNyZWF0ZXMgZHVtbXkgbm9kZXMgKGFuIGwtbGV2ZWwgbm9kZSB3aXRoIG1pbmltYWwgZGltZW5zaW9ucylcclxuICogZm9yIHRoZSBnaXZlbiBlZGdlIChvbmUgcGVyIGJlbmRwb2ludCkuIFRoZSBleGlzdGluZyBsLWxldmVsIHN0cnVjdHVyZVxyXG4gKiBpcyB1cGRhdGVkIGFjY29yZGluZ2x5LlxyXG4gKi9cclxuTGF5b3V0LnByb3RvdHlwZS5jcmVhdGVEdW1teU5vZGVzRm9yQmVuZHBvaW50cyA9IGZ1bmN0aW9uIChlZGdlKVxyXG57XHJcbiAgdmFyIGR1bW15Tm9kZXMgPSBbXTtcclxuICB2YXIgcHJldiA9IGVkZ2Uuc291cmNlO1xyXG5cclxuICB2YXIgZ3JhcGggPSB0aGlzLmdyYXBoTWFuYWdlci5jYWxjTG93ZXN0Q29tbW9uQW5jZXN0b3IoZWRnZS5zb3VyY2UsIGVkZ2UudGFyZ2V0KTtcclxuXHJcbiAgZm9yICh2YXIgaSA9IDA7IGkgPCBlZGdlLmJlbmRwb2ludHMubGVuZ3RoOyBpKyspXHJcbiAge1xyXG4gICAgLy8gY3JlYXRlIG5ldyBkdW1teSBub2RlXHJcbiAgICB2YXIgZHVtbXlOb2RlID0gdGhpcy5uZXdOb2RlKG51bGwpO1xyXG4gICAgZHVtbXlOb2RlLnNldFJlY3QobmV3IFBvaW50KDAsIDApLCBuZXcgRGltZW5zaW9uKDEsIDEpKTtcclxuXHJcbiAgICBncmFwaC5hZGQoZHVtbXlOb2RlKTtcclxuXHJcbiAgICAvLyBjcmVhdGUgbmV3IGR1bW15IGVkZ2UgYmV0d2VlbiBwcmV2IGFuZCBkdW1teSBub2RlXHJcbiAgICB2YXIgZHVtbXlFZGdlID0gdGhpcy5uZXdFZGdlKG51bGwpO1xyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIuYWRkKGR1bW15RWRnZSwgcHJldiwgZHVtbXlOb2RlKTtcclxuXHJcbiAgICBkdW1teU5vZGVzLmFkZChkdW1teU5vZGUpO1xyXG4gICAgcHJldiA9IGR1bW15Tm9kZTtcclxuICB9XHJcblxyXG4gIHZhciBkdW1teUVkZ2UgPSB0aGlzLm5ld0VkZ2UobnVsbCk7XHJcbiAgdGhpcy5ncmFwaE1hbmFnZXIuYWRkKGR1bW15RWRnZSwgcHJldiwgZWRnZS50YXJnZXQpO1xyXG5cclxuICB0aGlzLmVkZ2VUb0R1bW15Tm9kZXMucHV0KGVkZ2UsIGR1bW15Tm9kZXMpO1xyXG5cclxuICAvLyByZW1vdmUgcmVhbCBlZGdlIGZyb20gZ3JhcGggbWFuYWdlciBpZiBpdCBpcyBpbnRlci1ncmFwaFxyXG4gIGlmIChlZGdlLmlzSW50ZXJHcmFwaCgpKVxyXG4gIHtcclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlbW92ZShlZGdlKTtcclxuICB9XHJcbiAgLy8gZWxzZSwgcmVtb3ZlIHRoZSBlZGdlIGZyb20gdGhlIGN1cnJlbnQgZ3JhcGhcclxuICBlbHNlXHJcbiAge1xyXG4gICAgZ3JhcGgucmVtb3ZlKGVkZ2UpO1xyXG4gIH1cclxuXHJcbiAgcmV0dXJuIGR1bW15Tm9kZXM7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgY3JlYXRlcyBiZW5kcG9pbnRzIGZvciBlZGdlcyBmcm9tIHRoZSBkdW1teSBub2Rlc1xyXG4gKiBhdCBsLWxldmVsLlxyXG4gKi9cclxuTGF5b3V0LnByb3RvdHlwZS5jcmVhdGVCZW5kcG9pbnRzRnJvbUR1bW15Tm9kZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgdmFyIGVkZ2VzID0gW107XHJcbiAgZWRnZXMgPSBlZGdlcy5jb25jYXQodGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsRWRnZXMoKSk7XHJcbiAgZWRnZXMgPSB0aGlzLmVkZ2VUb0R1bW15Tm9kZXMua2V5U2V0KCkuY29uY2F0KGVkZ2VzKTtcclxuXHJcbiAgZm9yICh2YXIgayA9IDA7IGsgPCBlZGdlcy5sZW5ndGg7IGsrKylcclxuICB7XHJcbiAgICB2YXIgbEVkZ2UgPSBlZGdlc1trXTtcclxuXHJcbiAgICBpZiAobEVkZ2UuYmVuZHBvaW50cy5sZW5ndGggPiAwKVxyXG4gICAge1xyXG4gICAgICB2YXIgcGF0aCA9IHRoaXMuZWRnZVRvRHVtbXlOb2Rlcy5nZXQobEVkZ2UpO1xyXG5cclxuICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBwYXRoLmxlbmd0aDsgaSsrKVxyXG4gICAgICB7XHJcbiAgICAgICAgdmFyIGR1bW15Tm9kZSA9IHBhdGhbaV07XHJcbiAgICAgICAgdmFyIHAgPSBuZXcgUG9pbnREKGR1bW15Tm9kZS5nZXRDZW50ZXJYKCksXHJcbiAgICAgICAgICAgICAgICBkdW1teU5vZGUuZ2V0Q2VudGVyWSgpKTtcclxuXHJcbiAgICAgICAgLy8gdXBkYXRlIGJlbmRwb2ludCdzIGxvY2F0aW9uIGFjY29yZGluZyB0byBkdW1teSBub2RlXHJcbiAgICAgICAgdmFyIGVicCA9IGxFZGdlLmJlbmRwb2ludHMuZ2V0KGkpO1xyXG4gICAgICAgIGVicC54ID0gcC54O1xyXG4gICAgICAgIGVicC55ID0gcC55O1xyXG5cclxuICAgICAgICAvLyByZW1vdmUgdGhlIGR1bW15IG5vZGUsIGR1bW15IGVkZ2VzIGluY2lkZW50IHdpdGggdGhpc1xyXG4gICAgICAgIC8vIGR1bW15IG5vZGUgaXMgYWxzbyByZW1vdmVkICh3aXRoaW4gdGhlIHJlbW92ZSBtZXRob2QpXHJcbiAgICAgICAgZHVtbXlOb2RlLmdldE93bmVyKCkucmVtb3ZlKGR1bW15Tm9kZSk7XHJcbiAgICAgIH1cclxuXHJcbiAgICAgIC8vIGFkZCB0aGUgcmVhbCBlZGdlIHRvIGdyYXBoXHJcbiAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLmFkZChsRWRnZSwgbEVkZ2Uuc291cmNlLCBsRWRnZS50YXJnZXQpO1xyXG4gICAgfVxyXG4gIH1cclxufTtcclxuXHJcbkxheW91dC50cmFuc2Zvcm0gPSBmdW5jdGlvbiAoc2xpZGVyVmFsdWUsIGRlZmF1bHRWYWx1ZSwgbWluRGl2LCBtYXhNdWwpIHtcclxuICBpZiAobWluRGl2ICE9IHVuZGVmaW5lZCAmJiBtYXhNdWwgIT0gdW5kZWZpbmVkKSB7XHJcbiAgICB2YXIgdmFsdWUgPSBkZWZhdWx0VmFsdWU7XHJcblxyXG4gICAgaWYgKHNsaWRlclZhbHVlIDw9IDUwKVxyXG4gICAge1xyXG4gICAgICB2YXIgbWluVmFsdWUgPSBkZWZhdWx0VmFsdWUgLyBtaW5EaXY7XHJcbiAgICAgIHZhbHVlIC09ICgoZGVmYXVsdFZhbHVlIC0gbWluVmFsdWUpIC8gNTApICogKDUwIC0gc2xpZGVyVmFsdWUpO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICB2YXIgbWF4VmFsdWUgPSBkZWZhdWx0VmFsdWUgKiBtYXhNdWw7XHJcbiAgICAgIHZhbHVlICs9ICgobWF4VmFsdWUgLSBkZWZhdWx0VmFsdWUpIC8gNTApICogKHNsaWRlclZhbHVlIC0gNTApO1xyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiB2YWx1ZTtcclxuICB9XHJcbiAgZWxzZSB7XHJcbiAgICB2YXIgYSwgYjtcclxuXHJcbiAgICBpZiAoc2xpZGVyVmFsdWUgPD0gNTApXHJcbiAgICB7XHJcbiAgICAgIGEgPSA5LjAgKiBkZWZhdWx0VmFsdWUgLyA1MDAuMDtcclxuICAgICAgYiA9IGRlZmF1bHRWYWx1ZSAvIDEwLjA7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgIGEgPSA5LjAgKiBkZWZhdWx0VmFsdWUgLyA1MC4wO1xyXG4gICAgICBiID0gLTggKiBkZWZhdWx0VmFsdWU7XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIChhICogc2xpZGVyVmFsdWUgKyBiKTtcclxuICB9XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgZmluZHMgYW5kIHJldHVybnMgdGhlIGNlbnRlciBvZiB0aGUgZ2l2ZW4gbm9kZXMsIGFzc3VtaW5nXHJcbiAqIHRoYXQgdGhlIGdpdmVuIG5vZGVzIGZvcm0gYSB0cmVlIGluIHRoZW1zZWx2ZXMuXHJcbiAqL1xyXG5MYXlvdXQuZmluZENlbnRlck9mVHJlZSA9IGZ1bmN0aW9uIChub2Rlcylcclxue1xyXG4gIHZhciBsaXN0ID0gW107XHJcbiAgbGlzdCA9IGxpc3QuY29uY2F0KG5vZGVzKTtcclxuXHJcbiAgdmFyIHJlbW92ZWROb2RlcyA9IFtdO1xyXG4gIHZhciByZW1haW5pbmdEZWdyZWVzID0gbmV3IEhhc2hNYXAoKTtcclxuICB2YXIgZm91bmRDZW50ZXIgPSBmYWxzZTtcclxuICB2YXIgY2VudGVyTm9kZSA9IG51bGw7XHJcblxyXG4gIGlmIChsaXN0Lmxlbmd0aCA9PSAxIHx8IGxpc3QubGVuZ3RoID09IDIpXHJcbiAge1xyXG4gICAgZm91bmRDZW50ZXIgPSB0cnVlO1xyXG4gICAgY2VudGVyTm9kZSA9IGxpc3RbMF07XHJcbiAgfVxyXG5cclxuICBmb3IgKHZhciBpID0gMDsgaSA8IGxpc3QubGVuZ3RoOyBpKyspXHJcbiAge1xyXG4gICAgdmFyIG5vZGUgPSBsaXN0W2ldO1xyXG4gICAgdmFyIGRlZ3JlZSA9IG5vZGUuZ2V0TmVpZ2hib3JzTGlzdCgpLnNpemUoKTtcclxuICAgIHJlbWFpbmluZ0RlZ3JlZXMucHV0KG5vZGUsIG5vZGUuZ2V0TmVpZ2hib3JzTGlzdCgpLnNpemUoKSk7XHJcblxyXG4gICAgaWYgKGRlZ3JlZSA9PSAxKVxyXG4gICAge1xyXG4gICAgICByZW1vdmVkTm9kZXMucHVzaChub2RlKTtcclxuICAgIH1cclxuICB9XHJcblxyXG4gIHZhciB0ZW1wTGlzdCA9IFtdO1xyXG4gIHRlbXBMaXN0ID0gdGVtcExpc3QuY29uY2F0KHJlbW92ZWROb2Rlcyk7XHJcblxyXG4gIHdoaWxlICghZm91bmRDZW50ZXIpXHJcbiAge1xyXG4gICAgdmFyIHRlbXBMaXN0MiA9IFtdO1xyXG4gICAgdGVtcExpc3QyID0gdGVtcExpc3QyLmNvbmNhdCh0ZW1wTGlzdCk7XHJcbiAgICB0ZW1wTGlzdCA9IFtdO1xyXG5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbGlzdC5sZW5ndGg7IGkrKylcclxuICAgIHtcclxuICAgICAgdmFyIG5vZGUgPSBsaXN0W2ldO1xyXG5cclxuICAgICAgdmFyIGluZGV4ID0gbGlzdC5pbmRleE9mKG5vZGUpO1xyXG4gICAgICBpZiAoaW5kZXggPj0gMCkge1xyXG4gICAgICAgIGxpc3Quc3BsaWNlKGluZGV4LCAxKTtcclxuICAgICAgfVxyXG5cclxuICAgICAgdmFyIG5laWdoYm91cnMgPSBub2RlLmdldE5laWdoYm9yc0xpc3QoKTtcclxuXHJcbiAgICAgIGZvciAodmFyIGogaW4gbmVpZ2hib3Vycy5zZXQpXHJcbiAgICAgIHtcclxuICAgICAgICB2YXIgbmVpZ2hib3VyID0gbmVpZ2hib3Vycy5zZXRbal07XHJcbiAgICAgICAgaWYgKHJlbW92ZWROb2Rlcy5pbmRleE9mKG5laWdoYm91cikgPCAwKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgIHZhciBvdGhlckRlZ3JlZSA9IHJlbWFpbmluZ0RlZ3JlZXMuZ2V0KG5laWdoYm91cik7XHJcbiAgICAgICAgICB2YXIgbmV3RGVncmVlID0gb3RoZXJEZWdyZWUgLSAxO1xyXG5cclxuICAgICAgICAgIGlmIChuZXdEZWdyZWUgPT0gMSlcclxuICAgICAgICAgIHtcclxuICAgICAgICAgICAgdGVtcExpc3QucHVzaChuZWlnaGJvdXIpO1xyXG4gICAgICAgICAgfVxyXG5cclxuICAgICAgICAgIHJlbWFpbmluZ0RlZ3JlZXMucHV0KG5laWdoYm91ciwgbmV3RGVncmVlKTtcclxuICAgICAgICB9XHJcbiAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZW1vdmVkTm9kZXMgPSByZW1vdmVkTm9kZXMuY29uY2F0KHRlbXBMaXN0KTtcclxuXHJcbiAgICBpZiAobGlzdC5sZW5ndGggPT0gMSB8fCBsaXN0Lmxlbmd0aCA9PSAyKVxyXG4gICAge1xyXG4gICAgICBmb3VuZENlbnRlciA9IHRydWU7XHJcbiAgICAgIGNlbnRlck5vZGUgPSBsaXN0WzBdO1xyXG4gICAgfVxyXG4gIH1cclxuXHJcbiAgcmV0dXJuIGNlbnRlck5vZGU7XHJcbn07XHJcblxyXG4vKipcclxuICogRHVyaW5nIHRoZSBjb2Fyc2VuaW5nIHByb2Nlc3MsIHRoaXMgbGF5b3V0IG1heSBiZSByZWZlcmVuY2VkIGJ5IHR3byBncmFwaCBtYW5hZ2Vyc1xyXG4gKiB0aGlzIHNldHRlciBmdW5jdGlvbiBncmFudHMgYWNjZXNzIHRvIGNoYW5nZSB0aGUgY3VycmVudGx5IGJlaW5nIHVzZWQgZ3JhcGggbWFuYWdlclxyXG4gKi9cclxuTGF5b3V0LnByb3RvdHlwZS5zZXRHcmFwaE1hbmFnZXIgPSBmdW5jdGlvbiAoZ20pXHJcbntcclxuICB0aGlzLmdyYXBoTWFuYWdlciA9IGdtO1xyXG59O1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBMYXlvdXQ7XHJcbiIsImZ1bmN0aW9uIExheW91dENvbnN0YW50cygpIHtcbn1cblxuLyoqXG4gKiBMYXlvdXQgUXVhbGl0eVxuICovXG5MYXlvdXRDb25zdGFudHMuUFJPT0ZfUVVBTElUWSA9IDA7XG5MYXlvdXRDb25zdGFudHMuREVGQVVMVF9RVUFMSVRZID0gMTtcbkxheW91dENvbnN0YW50cy5EUkFGVF9RVUFMSVRZID0gMjtcblxuLyoqXG4gKiBEZWZhdWx0IHBhcmFtZXRlcnNcbiAqL1xuTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ1JFQVRFX0JFTkRTX0FTX05FRURFRCA9IGZhbHNlO1xuLy9MYXlvdXRDb25zdGFudHMuREVGQVVMVF9JTkNSRU1FTlRBTCA9IHRydWU7XG5MYXlvdXRDb25zdGFudHMuREVGQVVMVF9JTkNSRU1FTlRBTCA9IGZhbHNlO1xuTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX09OX0xBWU9VVCA9IHRydWU7XG5MYXlvdXRDb25zdGFudHMuREVGQVVMVF9BTklNQVRJT05fRFVSSU5HX0xBWU9VVCA9IGZhbHNlO1xuTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQU5JTUFUSU9OX1BFUklPRCA9IDUwO1xuTGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfVU5JRk9STV9MRUFGX05PREVfU0laRVMgPSBmYWxzZTtcblxuLy8gLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS1cbi8vIFNlY3Rpb246IEdlbmVyYWwgb3RoZXIgY29uc3RhbnRzXG4vLyAtLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuLypcbiAqIE1hcmdpbnMgb2YgYSBncmFwaCB0byBiZSBhcHBsaWVkIG9uIGJvdWRpbmcgcmVjdGFuZ2xlIG9mIGl0cyBjb250ZW50cy4gV2VcbiAqIGFzc3VtZSBtYXJnaW5zIG9uIGFsbCBmb3VyIHNpZGVzIHRvIGJlIHVuaWZvcm0uXG4gKi9cbkxheW91dENvbnN0YW50cy5ERUZBVUxUX0dSQVBIX01BUkdJTiA9IDEwO1xuXG4vKlxuICogVGhlIGhlaWdodCBvZiB0aGUgbGFiZWwgb2YgYSBjb21wb3VuZC4gV2UgYXNzdW1lIHRoZSBsYWJlbCBvZiBhIGNvbXBvdW5kXG4gKiBub2RlIGlzIHBsYWNlZCBhdCB0aGUgYm90dG9tIHdpdGggYSBkeW5hbWljIHdpZHRoIHNhbWUgYXMgdGhlIGNvbXBvdW5kXG4gKiBpdHNlbGYuXG4gKi9cbkxheW91dENvbnN0YW50cy5MQUJFTF9IRUlHSFQgPSAyMDtcblxuLypcbiAqIEFkZGl0aW9uYWwgbWFyZ2lucyB0aGF0IHdlIG1haW50YWluIGFzIHNhZmV0eSBidWZmZXIgZm9yIG5vZGUtbm9kZVxuICogb3ZlcmxhcHMuIENvbXBvdW5kIG5vZGUgbGFiZWxzIGFzIHdlbGwgYXMgZ3JhcGggbWFyZ2lucyBhcmUgaGFuZGxlZFxuICogc2VwYXJhdGVseSFcbiAqL1xuTGF5b3V0Q29uc3RhbnRzLkNPTVBPVU5EX05PREVfTUFSR0lOID0gNTtcblxuLypcbiAqIERlZmF1bHQgZGltZW5zaW9uIG9mIGEgbm9uLWNvbXBvdW5kIG5vZGUuXG4gKi9cbkxheW91dENvbnN0YW50cy5TSU1QTEVfTk9ERV9TSVpFID0gNDA7XG5cbi8qXG4gKiBEZWZhdWx0IGRpbWVuc2lvbiBvZiBhIG5vbi1jb21wb3VuZCBub2RlLlxuICovXG5MYXlvdXRDb25zdGFudHMuU0lNUExFX05PREVfSEFMRl9TSVpFID0gTGF5b3V0Q29uc3RhbnRzLlNJTVBMRV9OT0RFX1NJWkUgLyAyO1xuXG4vKlxuICogRW1wdHkgY29tcG91bmQgbm9kZSBzaXplLiBXaGVuIGEgY29tcG91bmQgbm9kZSBpcyBlbXB0eSwgaXRzIGJvdGhcbiAqIGRpbWVuc2lvbnMgc2hvdWxkIGJlIG9mIHRoaXMgdmFsdWUuXG4gKi9cbkxheW91dENvbnN0YW50cy5FTVBUWV9DT01QT1VORF9OT0RFX1NJWkUgPSA0MDtcblxuLypcbiAqIE1pbmltdW0gbGVuZ3RoIHRoYXQgYW4gZWRnZSBzaG91bGQgdGFrZSBkdXJpbmcgbGF5b3V0XG4gKi9cbkxheW91dENvbnN0YW50cy5NSU5fRURHRV9MRU5HVEggPSAxO1xuXG4vKlxuICogV29ybGQgYm91bmRhcmllcyB0aGF0IGxheW91dCBvcGVyYXRlcyBvblxuICovXG5MYXlvdXRDb25zdGFudHMuV09STERfQk9VTkRBUlkgPSAxMDAwMDAwO1xuXG4vKlxuICogV29ybGQgYm91bmRhcmllcyB0aGF0IHJhbmRvbSBwb3NpdGlvbmluZyBjYW4gYmUgcGVyZm9ybWVkIHdpdGhcbiAqL1xuTGF5b3V0Q29uc3RhbnRzLklOSVRJQUxfV09STERfQk9VTkRBUlkgPSBMYXlvdXRDb25zdGFudHMuV09STERfQk9VTkRBUlkgLyAxMDAwO1xuXG4vKlxuICogQ29vcmRpbmF0ZXMgb2YgdGhlIHdvcmxkIGNlbnRlclxuICovXG5MYXlvdXRDb25zdGFudHMuV09STERfQ0VOVEVSX1ggPSAxMjAwO1xuTGF5b3V0Q29uc3RhbnRzLldPUkxEX0NFTlRFUl9ZID0gOTAwO1xuXG5tb2R1bGUuZXhwb3J0cyA9IExheW91dENvbnN0YW50cztcbiIsInZhciBPcmdhbml6YXRpb24gPSByZXF1aXJlKCcuL09yZ2FuaXphdGlvbicpO1xyXG5cclxuZnVuY3Rpb24gTWVtYmVyUGFjayhjaGlsZEdyYXBoKVxyXG57XHJcbiAgICB2YXIgbWVtYmVycyA9IFtdIC8qQXJyYXlMaXN0PFNiZ25QRE5vZGU+Ki87XHJcbiAgICBtZW1iZXJzLmNvbmNhdChjaGlsZEdyYXBoLmdldE5vZGVzKCkpO1xyXG4gICAgXHJcbiAgICB2YXIgb3JnID0gbmV3IE9yZ2FuaXphdGlvbigpO1xyXG5cclxuICAgIHRoaXMubGF5b3V0KCk7XHJcblxyXG4gICAgdmFyIG5vZGVzID0gW107XHJcblxyXG4gICAgZm9yICh2YXIgaT0wOyBpPGNoaWxkR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGg7IGkrKylcclxuICAgIHtcclxuICAgICAgICBub2Rlc1tpXSA9IGNoaWxkR3JhcGguZ2V0Tm9kZXMoKVtpXTtcclxuICAgIH1cclxufVxyXG5cclxuTWVtYmVyUGFjay5wcm90b3R5cGUubGF5b3V0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGNvbXBhciA9IFtdIC8qbmV3IENvbXBhcmFibGVOb2RlW21lbWJlcnMuc2l6ZSgpXSovO1xyXG5cclxuICAgIHZhciBpPTA7XHJcbiAgICBmb3IgKHZhciBqPTA7IGo8dGhpcy5tZW1iZXJzLmxlbmd0aDsgaisrKVxyXG4gICAge1xyXG4gICAgICAgIGNvbXBhcltpKytdID0gbWVtYmVyc1tqXTtcclxuICAgIH1cclxuICAgIFxyXG4gICAgY29tcGFyLnNvcnQodGhpcy5jb21wYXJlTm9kZXMpO1xyXG4gICAgdGhpcy5tZW1iZXJzID0gW107XHJcbiAgICBcclxuICAgIGZvciAodmFyIGo9MDsgajxjb21wYXIubGVuZ3RoOyBqKyspXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5tZW1iZXJzLnB1c2goY29tcGFyW2pdLmdldE5vZGUoKSk7XHJcbiAgICB9XHJcbiAgICBcclxuICAgIGZvciAodmFyIGo9MDsgajx0aGlzLm1lbWJlcnMubGVuZ3RoOyBqKyspXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5vcmcuaW5zZXJ0Tm9kZShtZW1iZXJzW2pdKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBDb21wYWN0aW9uIGMgPSBuZXcgQ29tcGFjdGlvbihcclxuICAgIC8vIChBcnJheUxpc3Q8U2JnblBETm9kZT4pIG1lbWJlcnMpO1xyXG4gICAgLy8gYy5wZXJmb3JtKCk7XHJcblxyXG59O1xyXG5cclxuTWVtYmVyUGFjay5wcm90b3R5cGUuZ2V0V2lkdGggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy5vcmcuZ2V0V2lkdGgoKTtcclxufTtcclxuXHJcbk1lbWJlclBhY2sucHJvdG90eXBlLmdldEhlaWdodCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHJldHVybiB0aGlzLm9yZy5nZXRIZWlnaHQoKTtcclxufTtcclxuXHJcbk1lbWJlclBhY2sucHJvdG90eXBlLmFkanVzdExvY2F0aW9ucyA9IGZ1bmN0aW9uICh4LCB5KVxyXG57XHJcbiAgICB0aGlzLm9yZy5hZGp1c3RMb2NhdGlvbnMoeCwgeSk7XHJcbn07XHJcblxyXG5NZW1iZXJQYWNrLnByb3RvdHlwZS5nZXRNZW1iZXJzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMubWVtYmVycztcclxufTtcclxuXHJcbk1lbWJlclBhY2sucHJvdG90eXBlLmNvbXBhcmVOb2RlcyA9IGZ1bmN0aW9uIChub2RlQSwgbm9kZUIpXHJcbntcclxuICAgIHZhciBhU2l6ZSA9IG5vZGVBLmdldFdpZHRoKCkgKiBub2RlQS5nZXRIZWlnaHQoKTtcclxuICAgIHZhciBiU2l6ZSA9IG5vZGVCLmdldFdpZHRoKCkgKiBub2RlQi5nZXRIZWlnaHQoKTtcclxuICAgIFxyXG4gICAgaWYgKGFTaXplID4gYlNpemUpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIDE7XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmIChhU2l6ZSA8IGJTaXplKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiAtMTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gMDtcclxuICAgIH1cclxuICAgIHJldHVybiB0aGlzLm1lbWJlcnM7XHJcbn07XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IE1lbWJlclBhY2s7IiwidmFyIFNiZ25QRENvbnN0YW50cyA9IHJlcXVpcmUoJy4vU2JnblBEQ29uc3RhbnRzJyk7XHJcblxyXG4vKipcclxuKiBDcmVhdGVzIGEgY29udGFpbmVyIHdob3NlIHdpZHRoIGFuZCBoZWlnaHQgaXMgb25seSB0aGUgbWFyZ2luc1xyXG4qL1xyXG5mdW5jdGlvbiBPcmdhbml6YXRpb24oKVxyXG57XHJcbiAgICB0aGlzLndpZHRoID0gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTiAqIDI7XHJcbiAgICB0aGlzLmhlaWdodCA9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4gKiAyIDtcclxuXHJcbiAgICB0aGlzLnJvd1dpZHRoID0gW10gLypuZXcgQXJyYXlMaXN0PERvdWJsZT4oKSovO1xyXG4gICAgdGhpcy5yb3dIZWlnaHQgPSBbXSAvKm5ldyBBcnJheUxpc3Q8RG91YmxlPigpKi87XHJcblxyXG4gICAgdGhpcy5yb3dzID0gW1tdXTsvKm5ldyBBcnJheUxpc3Q8TGlua2VkTGlzdDxTYmduUEROb2RlPj4oKTsqL1xyXG59XHJcblxyXG5Pcmdhbml6YXRpb24ucHJvdG90eXBlLmdldFdpZHRoID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgc2hpZnRUb0xhc3RSb3coKTtcclxuICAgIHJldHVybiB0aGlzLndpZHRoO1xyXG59O1xyXG5cclxuT3JnYW5pemF0aW9uLnByb3RvdHlwZS5nZXRIZWlnaHQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy5oZWlnaHQ7XHJcbn07XHJcblxyXG4vKipcclxuKiBTY2FucyB0aGUgcm93V2lkdGggYXJyYXkgbGlzdCBhbmQgcmV0dXJucyB0aGUgaW5kZXggb2YgdGhlIHJvdyB0aGF0IGhhc1xyXG4qIHRoZSBtaW5pbXVtIHdpZHRoLlxyXG4qL1xyXG5Pcmdhbml6YXRpb24ucHJvdG90eXBlLmdldFNob3J0ZXN0Um93SW5kZXggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgciA9IC0xO1xyXG4gICAgdmFyIG1pbiA9IE51bWJlci5NQVhfVkFMVUU7XHJcbiAgICBcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy5yb3dzLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIGlmICh0aGlzLnJvd1dpZHRoW2ldIDwgbWluKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgciA9IGk7XHJcbiAgICAgICAgICAgIG1pbiA9IHRoaXMucm93V2lkdGhbaV07XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgcmV0dXJuIHI7XHJcbn07XHJcblxyXG4vKipcclxuKiBTY2FucyB0aGUgcm93V2lkdGggYXJyYXkgbGlzdCBhbmQgcmV0dXJucyB0aGUgaW5kZXggb2YgdGhlIHJvdyB0aGF0IGhhc1xyXG4qIHRoZSBtYXhpbXVtIHdpZHRoLlxyXG4qL1xyXG5Pcmdhbml6YXRpb24ucHJvdG90eXBlLmdldExvbmdlc3RSb3dJbmRleCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciByID0gLTE7XHJcbiAgICB2YXIgbWF4ID0gTnVtYmVyLk1BWF9WQUxVRTtcclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMucm93cy5sZW5ndGg7IGkrKylcclxuICAgIHtcclxuICAgICAgICBpZiAodGhpcy5yb3dXaWR0aFtpXSA+IG1heClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHIgPSBpO1xyXG4gICAgICAgICAgICBtYXggPSB0aGlzLnJvd1dpZHRoW2ldO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIHJldHVybiByO1xyXG59O1xyXG5cclxuT3JnYW5pemF0aW9uLnByb3RvdHlwZS5pbnNlcnROb2RlID0gZnVuY3Rpb24gKG5vZGUpXHJcbntcclxuICAgIGlmICh0aGlzLnJvd3MubGVuZ3RoIDw9IDApXHJcbiAgICB7XHJcbiAgICAgICAgaW5zZXJ0Tm9kZVRvUm93KG5vZGUsIDApO1xyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAoY2FuQWRkSG9yaXpvbnRhbChub2RlLmdldFdpZHRoKCksIG5vZGUuZ2V0SGVpZ2h0KCkpKVxyXG4gICAge1xyXG4gICAgICAgIGluc2VydE5vZGVUb1Jvdyhub2RlLCBnZXRTaG9ydGVzdFJvd0luZGV4KCkpO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIGluc2VydE5vZGVUb1Jvdyhub2RlLCB0aGlzLnJvd3MubGVuZ3RoKTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIHBlcmZvcm1zIHRpbGluZy4gSWYgYSBuZXcgcm93IGlzIG5lZWRlZCwgaXQgY3JlYXRlcyB0aGUgcm93XHJcbiogYW5kIHBsYWNlcyB0aGUgbmV3IG5vZGUgdGhlcmUuIE90aGVyd2lzZSwgaXQgcGxhY2VzIHRoZSBub2RlIHRvIHRoZSBlbmRcclxuKiBvZiB0aGUgZ2l2ZW4gcm93LlxyXG4qIFxyXG4qIEBwYXJhbSBub2RlXHJcbiogQHBhcmFtIHJvd0luZGV4XHJcbiovXHJcbk9yZ2FuaXphdGlvbi5wcm90b3R5cGUuaW5zZXJ0Tm9kZVRvUm93ID0gZnVuY3Rpb24gKG5vZGUsIHJvd0luZGV4KVxyXG57XHJcbiAgICAvLyBBZGQgbmV3IHJvdyBpZiBuZWVkZWRcclxuICAgIGlmIChyb3dJbmRleCA9PT0gdGhpcy5yb3dzLmxlbmd0aClcclxuICAgIHtcclxuICAgICAgICB0aGlzLnJvd3MucHVzaChbXSk7XHJcblxyXG4gICAgICAgIHRoaXMucm93V2lkdGgucHVzaChTYmduUERDb25zdGFudHMuQ09NUExFWF9NSU5fV0lEVEgpO1xyXG4gICAgICAgIHRoaXMucm93SGVpZ2h0LnB1c2goMC4wKTtcclxuXHJcbiAgICAgICAgLy8gVE9ETzogYXNzZXJ0IHJvd3Muc2l6ZSgpID09IHJvd1dpZHRoLnNpemUoKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBVcGRhdGUgcm93IHdpZHRoXHJcbiAgICB2YXIgdyA9IHRoaXMucm93V2lkdGhbcm93SW5kZXhdICsgbm9kZS5nZXRXaWR0aCgpO1xyXG5cclxuICAgIGlmICh0aGlzLnJvd3Nbcm93SW5kZXhdID4gMClcclxuICAgIHtcclxuICAgICAgICB3ICs9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUjtcclxuICAgIH1cclxuICAgIFxyXG4gICAgdGhpcy5yb3dXaWR0aC5zZXRbcm93SW5kZXhdID0gdztcclxuXHJcbiAgICAvLyBVcGRhdGUgY29tcGxleCB3aWR0aFxyXG4gICAgaWYgKHRoaXMud2lkdGggPCB3KVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMud2lkdGggPSB3O1xyXG4gICAgfVxyXG5cclxuICAgIC8vIFVwZGF0ZSBoZWlnaHRcclxuICAgIHZhciBoID0gbm9kZS5nZXRIZWlnaHQoKTtcclxuICAgIGlmIChyb3dJbmRleCA+IDApXHJcbiAgICB7XHJcbiAgICAgICAgaCArPSBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fVkVSVElDQUxfQlVGRkVSO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB2YXIgZXh0cmFIZWlnaHQgPSAwO1xyXG4gICAgaWYgKGggPiB0aGlzLnJvd0hlaWdodFtyb3dJbmRleF0pXHJcbiAgICB7XHJcbiAgICAgICAgZXh0cmFIZWlnaHQgPSB0aGlzLnJvd0hlaWdodFtyb3dJbmRleF07XHJcbiAgICAgICAgdGhpcy5yb3dIZWlnaHRbcm93SW5kZXhdID0gaDtcclxuICAgICAgICBleHRyYUhlaWdodCA9IHRoaXMucm93SGVpZ2h0W3Jvd0luZGV4XSAtIGV4dHJhSGVpZ2h0O1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMuaGVpZ2h0ICs9IGV4dHJhSGVpZ2h0O1xyXG5cclxuICAgIC8vIEluc2VydCBub2RlXHJcbiAgICB0aGlzLnJvd3Nbcm93SW5kZXhdLnB1c2gobm9kZSk7XHJcbn07XHJcblxyXG4vKipcclxuKiBJZiBtb3ZpbmcgdGhlIGxhc3Qgbm9kZSBmcm9tIHRoZSBsb25nZXN0IHJvdyBhbmQgYWRkaW5nIGl0IHRvIHRoZSBsYXN0XHJcbiogcm93IG1ha2VzIHRoZSBib3VuZGluZyBib3ggc21hbGxlciwgZG8gaXQuXHJcbiovXHJcbk9yZ2FuaXphdGlvbi5wcm90b3R5cGUuc2hpZnRUb0xhc3RSb3cgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgbG9uZ2VzdCA9IHRoaXMuZ2V0TG9uZ2VzdFJvd0luZGV4KCk7XHJcbiAgICB2YXIgbGFzdCA9IHRoaXMucm93V2lkdGgubGVuZ3RoIC0gMTtcclxuICAgIHZhciByb3cgPSByb3dzW2xvbmdlc3RdOyAvKkxpbmtlZExpc3Q8U2JnblBETm9kZT4qL1xyXG4gICAgdmFyIG5vZGUgPSByb3dbKHJvdy5sZW5ndGgtMSldO1xyXG5cclxuICAgIHZhciBkaWZmID0gbm9kZS5nZXRXaWR0aCgpICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX0hPUklaT05UQUxfQlVGRkVSO1xyXG5cclxuICAgIGlmICh0aGlzLndpZHRoIC0gdGhpcy5yb3dXaWR0aFsodGhpcy5yb3dXaWR0aC5sZW5ndGgtMSldID4gZGlmZiAmJiBcclxuICAgICAgICB0aGlzLnJvd0hlaWdodFsodGhpcy5yb3dIZWlnaHQubGVuZ3RoLTEpXSA+IG5vZGUuZ2V0SGVpZ2h0KCkpXHJcbiAgICB7XHJcbiAgICAgICAgcm93LnBvcCgpO1xyXG4gICAgICAgIHRoaXMucm93c1t0aGlzLnJvd3MubGVuZ3RoXS5wdXNoKG5vZGUpO1xyXG4gICAgICAgIHRoaXMucm93V2lkdGhbbG9uZ2VzdF0gPSB0aGlzLnJvd1dpZHRoW2xvbmdlc3RdIC0gZGlmZjtcclxuICAgICAgICB0aGlzLnJvd1dpZHRoW2xhc3RdID0gdGhpcy5yb3dXaWR0aFtsYXN0XSArIGRpZmY7XHJcblxyXG4gICAgICAgIHRoaXMud2lkdGggPSB0aGlzLnJvd1dpZHRoW3RoaXMuZ2V0TG9uZ2VzdFJvd0luZGV4KCldO1xyXG5cclxuICAgICAgICAvLyBVcGRhdGUgaGVpZ2h0IG9mIHRoZSBvcmdhbml6YXRpb25cclxuICAgICAgICB2YXIgbWF4SGVpZ2h0ID0gTnVtYmVyLk1JTl9WQUxVRTtcclxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJvdy5sZW5ndGg7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmIChyb3dbaV0uZ2V0SGVpZ2h0KCkgPiBtYXhIZWlnaHQpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIG1heEhlaWdodCA9IHJvd1tpXS5nZXRIZWlnaHQoKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICBpZiAobG9uZ2VzdCA+IDApXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBtYXhIZWlnaHQgKz0gU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX1ZFUlRJQ0FMX0JVRkZFUjtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHZhciBwcmV2VG90YWwgPSB0aGlzLnJvd0hlaWdodFtsb25nZXN0XSArIHRoaXMucm93SGVpZ2h0W2xhc3RdO1xyXG5cclxuICAgICAgICB0aGlzLnJvd0hlaWdodFtsb25nZXN0XSA9IG1heEhlaWdodDtcclxuICAgICAgICBpZiAodGhpcy5yb3dIZWlnaHRbbGFzdF0gPCBcclxuICAgICAgICAgICAgbm9kZS5nZXRIZWlnaHQoKSArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVIpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLnJvd0hlaWdodFtsYXN0XSA9XHJcbiAgICAgICAgICAgICAgICAgICAgbm9kZS5nZXRIZWlnaHQoKSArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVI7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICB2YXIgZmluYWxUb3RhbCA9IHRoaXMucm93SGVpZ2h0W2xvbmdlc3RdICsgdGhpcy5yb3dIZWlnaHRbbGFzdF07XHJcbiAgICAgICAgdGhpcy5oZWlnaHQgKz0gKGZpbmFsVG90YWwgLSBwcmV2VG90YWwpO1xyXG5cclxuICAgICAgICB0aGlzLnNoaWZ0VG9MYXN0Um93KCk7XHJcbiAgICB9XHJcbn07XHJcblxyXG5Pcmdhbml6YXRpb24ucHJvdG90eXBlLmNhbkFkZEhvcml6b250YWwgPSBmdW5jdGlvbiAoZXh0cmFXaWR0aCwgZXh0cmFIZWlnaHQpXHJcbntcclxuICAgIHZhciBzcmkgPSB0aGlzLmdldFNob3J0ZXN0Um93SW5kZXgoKTtcclxuXHJcbiAgICBpZiAoc3JpIDwgMClcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIFxyXG4gICAgdmFyIG1pbiA9IHRoaXMucm93V2lkdGhbc3JpXTtcclxuICAgIHZhciBoRGlmZiA9IDA7XHJcbiAgICBpZiAodGhpcy5yb3dIZWlnaHRbc3JpXSA8IGV4dHJhSGVpZ2h0KVxyXG4gICAge1xyXG4gICAgICAgIGlmIChzcmkgPiAwKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaERpZmYgPSBleHRyYUhlaWdodCArIFxyXG4gICAgICAgICAgICAgICAgICAgIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVIgLSBcclxuICAgICAgICAgICAgICAgICAgICB0aGlzLnJvd0hlaWdodFtzcmldO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIGlmICgodGhpcy53aWR0aCAtIG1pbikgPj0gXHJcbiAgICAgICAgKGV4dHJhV2lkdGggKyBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fSE9SSVpPTlRBTF9CVUZGRVIpKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiAodGhpcy5oZWlnaHQgKyBoRGlmZikgPiBcclxuICAgICAgICAgICAobWluICsgZXh0cmFXaWR0aCArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUik7XHJcbn07XHJcblxyXG5Pcmdhbml6YXRpb24ucHJvdG90eXBlLmFkanVzdExvY2F0aW9ucyA9IGZ1bmN0aW9uICh4LCB5KVxyXG57XHJcbiAgICB4ICs9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU47XHJcbiAgICB5ICs9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU47XHJcblxyXG4gICAgdmFyIGxlZnQgPSB4O1xyXG5cclxuICAgIGZvciAodmFyIGk9MDsgaTx0aGlzLnJvd3MubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHJvdyA9IHRoaXMucm93c1tpXTtcclxuICAgICAgICBcclxuICAgICAgICB4ID0gbGVmdDtcclxuICAgICAgICB2YXIgbWF4SGVpZ2h0ID0gMDtcclxuICAgICAgICBmb3IgKHZhciBqPTA7IGo8cm93Lmxlbmd0aDsgaisrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdmFyIG5vZGUgPSByb3dbal07XHJcbiAgICAgICAgICAgIG5vZGUuc2V0TG9jYXRpb24oeCwgeSk7XHJcblxyXG4gICAgICAgICAgICB4ICs9IChub2RlLmdldFdpZHRoKCkgKyBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fSE9SSVpPTlRBTF9CVUZGRVIpO1xyXG5cclxuICAgICAgICAgICAgaWYgKG5vZGUuZ2V0SGVpZ2h0KCkgPiBtYXhIZWlnaHQpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIG1heEhlaWdodCA9IG5vZGUuZ2V0SGVpZ2h0KCk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHkgKz0gKG1heEhlaWdodCArIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9WRVJUSUNBTF9CVUZGRVIpO1xyXG4gICAgfVxyXG59O1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBPcmdhbml6YXRpb247XHJcbiIsIi8qXG4gKlRoaXMgY2xhc3MgaXMgdGhlIGphdmFzY3JpcHQgaW1wbGVtZW50YXRpb24gb2YgdGhlIFBvaW50LmphdmEgY2xhc3MgaW4gamRrXG4gKi9cbmZ1bmN0aW9uIFBvaW50KHgsIHksIHApIHtcbiAgdGhpcy54ID0gbnVsbDtcbiAgdGhpcy55ID0gbnVsbDtcbiAgaWYgKHggPT0gbnVsbCAmJiB5ID09IG51bGwgJiYgcCA9PSBudWxsKSB7XG4gICAgdGhpcy54ID0gMDtcbiAgICB0aGlzLnkgPSAwO1xuICB9XG4gIGVsc2UgaWYgKHR5cGVvZiB4ID09ICdudW1iZXInICYmIHR5cGVvZiB5ID09ICdudW1iZXInICYmIHAgPT0gbnVsbCkge1xuICAgIHRoaXMueCA9IHg7XG4gICAgdGhpcy55ID0geTtcbiAgfVxuICBlbHNlIGlmICh4LmNvbnN0cnVjdG9yLm5hbWUgPT0gJ1BvaW50JyAmJiB5ID09IG51bGwgJiYgcCA9PSBudWxsKSB7XG4gICAgcCA9IHg7XG4gICAgdGhpcy54ID0gcC54O1xuICAgIHRoaXMueSA9IHAueTtcbiAgfVxufVxuXG5Qb2ludC5wcm90b3R5cGUuZ2V0WCA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIHRoaXMueDtcbn1cblxuUG9pbnQucHJvdG90eXBlLmdldFkgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiB0aGlzLnk7XG59XG5cblBvaW50LnByb3RvdHlwZS5nZXRMb2NhdGlvbiA9IGZ1bmN0aW9uICgpIHtcbiAgcmV0dXJuIG5ldyBQb2ludCh0aGlzLngsIHRoaXMueSk7XG59XG5cblBvaW50LnByb3RvdHlwZS5zZXRMb2NhdGlvbiA9IGZ1bmN0aW9uICh4LCB5LCBwKSB7XG4gIGlmICh4LmNvbnN0cnVjdG9yLm5hbWUgPT0gJ1BvaW50JyAmJiB5ID09IG51bGwgJiYgcCA9PSBudWxsKSB7XG4gICAgcCA9IHg7XG4gICAgdGhpcy5zZXRMb2NhdGlvbihwLngsIHAueSk7XG4gIH1cbiAgZWxzZSBpZiAodHlwZW9mIHggPT0gJ251bWJlcicgJiYgdHlwZW9mIHkgPT0gJ251bWJlcicgJiYgcCA9PSBudWxsKSB7XG4gICAgLy9pZiBib3RoIHBhcmFtZXRlcnMgYXJlIGludGVnZXIganVzdCBtb3ZlICh4LHkpIGxvY2F0aW9uXG4gICAgaWYgKHBhcnNlSW50KHgpID09IHggJiYgcGFyc2VJbnQoeSkgPT0geSkge1xuICAgICAgdGhpcy5tb3ZlKHgsIHkpO1xuICAgIH1cbiAgICBlbHNlIHtcbiAgICAgIHRoaXMueCA9IE1hdGguZmxvb3IoeCArIDAuNSk7XG4gICAgICB0aGlzLnkgPSBNYXRoLmZsb29yKHkgKyAwLjUpO1xuICAgIH1cbiAgfVxufVxuXG5Qb2ludC5wcm90b3R5cGUubW92ZSA9IGZ1bmN0aW9uICh4LCB5KSB7XG4gIHRoaXMueCA9IHg7XG4gIHRoaXMueSA9IHk7XG59XG5cblBvaW50LnByb3RvdHlwZS50cmFuc2xhdGUgPSBmdW5jdGlvbiAoZHgsIGR5KSB7XG4gIHRoaXMueCArPSBkeDtcbiAgdGhpcy55ICs9IGR5O1xufVxuXG5Qb2ludC5wcm90b3R5cGUuZXF1YWxzID0gZnVuY3Rpb24gKG9iaikge1xuICBpZiAob2JqLmNvbnN0cnVjdG9yLm5hbWUgPT0gXCJQb2ludFwiKSB7XG4gICAgdmFyIHB0ID0gb2JqO1xuICAgIHJldHVybiAodGhpcy54ID09IHB0LngpICYmICh0aGlzLnkgPT0gcHQueSk7XG4gIH1cbiAgcmV0dXJuIHRoaXMgPT0gb2JqO1xufVxuXG5Qb2ludC5wcm90b3R5cGUudG9TdHJpbmcgPSBmdW5jdGlvbiAoKSB7XG4gIHJldHVybiBuZXcgUG9pbnQoKS5jb25zdHJ1Y3Rvci5uYW1lICsgXCJbeD1cIiArIHRoaXMueCArIFwiLHk9XCIgKyB0aGlzLnkgKyBcIl1cIjtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBQb2ludDtcbiIsImZ1bmN0aW9uIFBvaW50RCh4LCB5KSB7XG4gIGlmICh4ID09IG51bGwgJiYgeSA9PSBudWxsKSB7XG4gICAgdGhpcy54ID0gMDtcbiAgICB0aGlzLnkgPSAwO1xuICB9IGVsc2Uge1xuICAgIHRoaXMueCA9IHg7XG4gICAgdGhpcy55ID0geTtcbiAgfVxufVxuXG5Qb2ludEQucHJvdG90eXBlLmdldFggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy54O1xufTtcblxuUG9pbnRELnByb3RvdHlwZS5nZXRZID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMueTtcbn07XG5cblBvaW50RC5wcm90b3R5cGUuc2V0WCA9IGZ1bmN0aW9uICh4KVxue1xuICB0aGlzLnggPSB4O1xufTtcblxuUG9pbnRELnByb3RvdHlwZS5zZXRZID0gZnVuY3Rpb24gKHkpXG57XG4gIHRoaXMueSA9IHk7XG59O1xuXG5Qb2ludEQucHJvdG90eXBlLmdldERpZmZlcmVuY2UgPSBmdW5jdGlvbiAocHQpXG57XG4gIHJldHVybiBuZXcgRGltZW5zaW9uRCh0aGlzLnggLSBwdC54LCB0aGlzLnkgLSBwdC55KTtcbn07XG5cblBvaW50RC5wcm90b3R5cGUuZ2V0Q29weSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiBuZXcgUG9pbnREKHRoaXMueCwgdGhpcy55KTtcbn07XG5cblBvaW50RC5wcm90b3R5cGUudHJhbnNsYXRlID0gZnVuY3Rpb24gKGRpbSlcbntcbiAgdGhpcy54ICs9IGRpbS53aWR0aDtcbiAgdGhpcy55ICs9IGRpbS5oZWlnaHQ7XG4gIHJldHVybiB0aGlzO1xufTtcblxubW9kdWxlLmV4cG9ydHMgPSBQb2ludEQ7XG4iLCJmdW5jdGlvbiBQb2x5b21pbm8oKSBcbntcbiAgICAvLyB0aGUgbnVtYmVyIG9mIGNlbGxzXG4gICAgdGhpcy5sID0gMDtcblxuICAgIC8vdGhlIHJlc3VsdGluZyBwbGFjZW1lbnQgY29vcmRpbmF0ZXNcbiAgICB0aGlzLnggPSAwO1xuICAgIHRoaXMueSA9IDA7XG5cbiAgICB0aGlzLmxhYmVsID0gXCJcIjtcblxuICAgIC8vIHBvbHlvbWlubyBjZWxsc1xuICAgIHRoaXMuY29vcmQgPSBbXTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBQb2x5b21pbm87IiwidmFyIFBvbHlvbWlub1F1aWNrU29ydCA9IHJlcXVpcmUoJy4vUG9seW9taW5vUXVpY2tTb3J0Jyk7XG52YXIgSW50ZWdlciA9IHJlcXVpcmUoJy4vSW50ZWdlcicpO1xudmFyIFJlY3RhbmdsZUQgPSByZXF1aXJlKCcuL1JlY3RhbmdsZUQnKTtcblxuZnVuY3Rpb24gUG9seW9taW5vUGFja2luZygpXG57XG4gICAgLy8gUG9seW9taW5vIGFycmF5XG4gICB0aGlzLnBvbHlvbWlub2VzID0gW107XG5cbiAgIC8vIEJvdW5kaW5nIHJlY3RhbmdsZXMgb2YgdGhlIHBvbHlvbWlub2VzXG4gICB0aGlzLnJlY3QgPSBbXTtcblxuICAgLy8gVGhlIGdyaWRcbiAgIHRoaXMuZ3JpZCA9IFtbXV07XG5cbiAgIC8vIENlbnRlciBwb2ludCBvZiB0aGUgZ3JpZFxuICAgdGhpcy5nY3ggPSAwO1xuICAgdGhpcy5nY3kgPSAwO1xuXG4gICAvLyBHcmlkIHNpemVcbiAgIHRoaXMuc2l6ZVggPSAwO1xuICAgdGhpcy5zaXplWSA9IDA7XG5cbiAgIC8vIFRoZSBudW1iZXIgb2YgYWxyZWFkeSBwbGFjZWQgcG9seW9taW5vZXNcbiAgIHRoaXMuY3VybWlubyA9IDA7XG5cbiAgIC8vIFN0b3JlcyB0aGUgb3JkZXJpbmcgb2YgdGhlIHBvbHlvbWlub2VzXG4gICB0aGlzLmluZCA9IFtdO1xuXG4gICAvLyBSYW5kb20gZ2VuZXJhdG9yXG4gICAvL1RPRE86IFJhbmRvbSBSZ2VuO1xufVxuXG4vKipcbiogVGhpcyBtZXRob2QgcGVyZm9ybXMgcG9seW9taW5vIHBhY2tpbmcuXG4qL1xuUG9seW9taW5vUGFja2luZy5wcm90b3R5cGUucGFjayA9IGZ1bmN0aW9uIChwbSwgcGNvdW50KVxue1xuICAgIHRoaXMucG9seW9taW5vZXMgPSBwbTtcbiAgICB0aGlzLnJlY3QgPSBbXTtcblxuICAgIC8vIG1ha2UgdGhlIGluaXRpYWwgZ3JpZFxuICAgIHRoaXMubWFrZUdyaWQoMTAwLCAxMDAsIDApO1xuXG4gICAgLy8gbWFrZSB0aGUgcmFuZG9tIHBlcm11dGF0aW9uIG9mIHBvbHlvbWlubyBjZWxscyBhbmRcbiAgICAvLyBjYWxjdWxhdGUgdGhlIGJvdW5kaW5nIHJlY3RhbmdsZXMuXG4gICAgLy8gVE9ETzogUmdlbiA9IG5ldyBSYW5kb20oMSk7XG4gICAgZm9yICh2YXIgayA9IDA7IGsgPCBwY291bnQ7IGsrKylcbiAgICB7XG4gICAgICAgIHRoaXMuUmFuZG9taXplTWlubyhrKTtcbiAgICB9XG4gICAgXG4gICAgLy8gb3JkZXIgdGhlIHBvbHlvbWlub2VzIGluIGluY3JlYXNpbmcgc2l6ZVxuICAgIHZhciBrZXkgPSBbXTtcbiAgICB0aGlzLmluZCA9IFtdO1xuXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBwY291bnQ7IGkrKylcbiAgICB7XG4gICAgICAgIGtleVtpXSA9IC0odGhpcy5yZWN0W2ldLmdldE1heFgoKSAtIHRoaXMucmVjdFtpXS5nZXRNaW5YKCkpXG4gICAgICAgICAgICAgICAgICAgICAgICAtICh0aGlzLnJlY3RbaV0uZ2V0TWF4WSgpIC0gdGhpcy5yZWN0W2ldLmdldE1pblkoKSk7XG4gICAgfVxuICAgIHZhciBxc29ydCA9IG5ldyBQb2x5b21pbm9RdWlja1NvcnQoKTtcbiAgICBxc29ydC5zb3J0KHBjb3VudCwga2V5LCB0aGlzLmluZCk7XG5cbiAgICAvLyBwbGFjZSBvbmUgYnkgb25lIHN0YXJ0aW5nIGZyb20gdGhlIGxhcmdlc3RcbiAgICBmb3IgKHRoaXMuY3VybWlubyA9IDA7IHRoaXMuY3VybWlubyA8IHBjb3VudDsgdGhpcy5jdXJtaW5vKyspXG4gICAge1xuICAgICAgICBwdXRNaW5vKHRoaXMuaW5kW3RoaXMuY3VybWlub10pO1xuICAgIH1cbn07XG5cbi8qKlxuKiBUaGlzIGNyZWF0ZXMgdGhlIGdyaWQgb2YgZ2l2ZW4gZGltZW5zaW9ucyBhbmQgZmlsbHMgaXQgd2l0aCB0aGUgYWxyZWFkeVxuKiBwbGFjZWQgcG9seW9taW5vZXMuXG4qL1xuUG9seW9taW5vUGFja2luZy5wcm90b3R5cGUubWFrZUdyaWQgPSBmdW5jdGlvbiAoZGlteCwgZGlteSwgbU4pXG57XG4gICAgdmFyIGk7XG5cbiAgICAvLyBhbGxvY2F0ZSB0aGUgZ3JpZFxuICAgIC8qIFRPRE86IFdlIGRvbid0IG5lZWQgdGhpcyEgXG4gICAgdGhpcy5ncmlkID0gW1tdXTtcbiAgICBmb3IgKGkgPSAwOyBpIDwgZGlteTsgaSsrKVxuICAgIHtcbiAgICAgICAgdGhpcy5ncmlkW2ldID0gbmV3IGJ5dGVbZGlteF07XG4gICAgfSovXG5cbiAgICB2YXIgZHggPSBkaW14IC8gMiAtIHRoaXMuZ2N4O1xuICAgIHZhciBkeSA9IGRpbXkgLyAyIC0gdGhpcy5nY3k7XG4gICAgdGhpcy5nY3ggPSBkaW14IC8gMjtcbiAgICB0aGlzLmdjeSA9IGRpbXkgLyAyO1xuICAgIHRoaXMuc2l6ZVggPSBkaW14O1xuICAgIHRoaXMuc2l6ZVkgPSBkaW15O1xuXG4gICAgLy8gbWFyayB0aGUgcG9zaXRpb25zIG9jY3VwaWVkIHdpdGggdGhlIGFscmVhZHkgcGxhY2VkXG4gICAgLy8gcG9seW9taW5vZXMuXG4gICAgZm9yIChpID0gMDsgaSA8IG1OOyBpKyspXG4gICAge1xuICAgICAgICB2YXIgcCA9IHRoaXMucG9seW9taW5vZXNbdGhpcy5pbmRbaV1dO1xuICAgICAgICBwLnggKz0gZHg7XG4gICAgICAgIHAueSArPSBkeTtcblxuICAgICAgICBmb3IgKHZhciBrID0gMDsgayA8IHAubDsgaysrKVxuICAgICAgICB7XG4gICAgICAgICAgICB2YXIgeHggPSBwLmNvb3JkW2tdLmdldFgoKSArIHAueDtcbiAgICAgICAgICAgIHZhciB5eSA9IHAuY29vcmRba10uZ2V0WSgpICsgcC55O1xuICAgICAgICAgICAgdGhpcy5ncmlkW3l5XVt4eF0gPSAxO1xuICAgICAgICB9XG4gICAgfVxufTtcblxuLyoqXG4qIFRoaXMgbWV0aG9kIGNoZWNrcyB3aGV0aGVyIHAgY2FuIGJlIHBsYWNlZCBpbiAoeCx5KS4gRm9yIGVhY2ggcG9seW9taW5vLFxuKiBjaGVjayBpZiB0aGUgKHgseSkgaXMgb2NjdXBpZWQvZml0cyBpbiB0aGUgZ3JpZC5cbiovXG5Qb2x5b21pbm9QYWNraW5nLnByb3RvdHlwZS5Jc0ZyZWVQbGFjZSA9IGZ1bmN0aW9uICh4LCB5LCBwKVxue1xuICAgIGZvciAodmFyIGsgPSAwOyBrIDwgcC5sOyBrKyspXG4gICAge1xuICAgICAgICB2YXIgeHggPSBwLmNvb3JkW2tdLmdldFgoKSArIHg7XG4gICAgICAgIHZhciB5eSA9IHAuY29vcmRba10uZ2V0WSgpICsgeTtcbiAgICAgICAgXG4gICAgICAgIC8vIHJldHVybiBmYWxzZSBpZiB0aGUgcG9seW9taW5vIGdvZXMgb3V0c2lkZSB0aGUgZ3JpZFxuICAgICAgICBpZiAoeHggPCAwIHx8IHl5IDwgMCB8fCB4eCA+PSB0aGlzLnNpemVYIHx8IHl5ID49IHRoaXMuc2l6ZVkpXG4gICAgICAgIHtcbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuICAgICAgICBcbiAgICAgICAgLy8gb3IgdGhlIHBvc2l0aW9uIGlzIG9jY3VwaWVkXG4gICAgICAgIGlmIChncmlkW3l5XVt4eF0gIT09IDApXG4gICAgICAgIHtcbiAgICAgICAgICAgIHJldHVybiBmYWxzZTtcbiAgICAgICAgfVxuICAgIH1cblxuICAgIC8vIHJlbWVtYmVyIHRoZSBwb3NpdGlvblxuICAgIHAueCA9IHg7XG4gICAgcC55ID0geTtcbiAgICByZXR1cm4gdHJ1ZTtcbn07XG5cbi8qKlxuKiBUaGlzIHRyaWVzIHRvIGZpbmQgYSBmcmVlIHBsYWNlIGluIHRoZSBncmlkLiBUaGUgZnVuY3Rpb24gcmV0dXJucyB0cnVlIGlmXG4qIHRoZSBwbGFjZW1lbnQgaXMgc3VjY2Vzc2Z1bC5cbiovXG5Qb2x5b21pbm9QYWNraW5nLnByb3RvdHlwZS50cnlQbGFjaW5nID0gZnVuY3Rpb24gKHBpKVxue1xuICAgIHZhciBwID0gdGhpcy5wb2x5b21pbm9lc1twaV07XG5cbiAgICB2YXIgY3ggPSB0aGlzLmdjeCAtICh0aGlzLnJlY3RbcGldLmdldE1heFgoKSArIHRoaXMucmVjdFtwaV0uZ2V0TWluWCgpKSAvIDI7XG4gICAgdmFyIGN5ID0gdGhpcy5nY3kgLSAodGhpcy5yZWN0W3BpXS5nZXRNYXhZKCkgKyB0aGlzLnJlY3RbcGldLmdldE1pblkoKSkgLyAyO1xuXG4gICAgLy8gc2VlIGlmIHRoZSBjZW50ZXIgcG9pbnQgaXMgbm90IG9jY3VwaWVkXG4gICAgaWYgKHRoaXMuSXNGcmVlUGxhY2UoY3gsIGN5LCBwKSlcbiAgICB7XG4gICAgICAgIHJldHVybiB0cnVlO1xuICAgIH1cblxuICAgIC8vIHRyeSBwbGFjaW5nIGluIHRoZSBpbmNyZWFzaW5nIGRpc3RhbmNlIGZyb20gdGhlIGNlbnRlclxuICAgIGZvciAodmFyIGQgPSAxOyBkIDwgdGhpcy5zaXplWCAvIDI7IGQrKylcbiAgICB7XG4gICAgICAgIGZvciAodmFyIGkgPSAtZDsgaSA8IGQ7IGkrKylcbiAgICAgICAge1xuICAgICAgICAgICAgdmFyIGkxID0gKGkgKyBkICsgMSkgLyAyICogKCgoaSAmIDEpID09PSAxKSA/IDEgOiAtMSk7XG4gICAgICAgICAgICBpZiAodGhpcy5Jc0ZyZWVQbGFjZSgtZCArIGN4LCAtaTEgKyBjeSwgcCkpXG4gICAgICAgICAgICAgICAgICAgIHJldHVybiB0cnVlO1xuICAgICAgICAgICAgaWYgKHRoaXMuSXNGcmVlUGxhY2UoZCArIGN4LCBpMSArIGN5LCBwKSlcbiAgICAgICAgICAgICAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgICAgICAgICBpZiAodGhpcy5Jc0ZyZWVQbGFjZShjeCAtIGkxLCBkICsgY3ksIHApKVxuICAgICAgICAgICAgICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgICAgICAgIGlmICh0aGlzLklzRnJlZVBsYWNlKGkxICsgY3gsIC1kICsgY3ksIHApKVxuICAgICAgICAgICAgICAgICAgICByZXR1cm4gdHJ1ZTtcbiAgICAgICAgfVxuICAgIH1cbiAgICByZXR1cm4gZmFsc2U7XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgcGxhY2VzIHRoZSBnaXZlbiBwb2x5b21pbm8uIFRoZSBncmlkIGlzIGVubGFyZ2VkIGlmXG4qIG5lY2Vzc2FyeS5cbiovXG5Qb2x5b21pbm9QYWNraW5nLnByb3RvdHlwZS5wdXRNaW5vID0gZnVuY3Rpb24gKHBpKVxue1xuICAgIHZhciBwID0gdGhpcy5wb2x5b21pbm9lc1twaV07XG5cbiAgICAvLyBpZiB0aGUgcG9seW9taW5vIGNhbm5vdCBiZSBwbGFjZWQgaW4gdGhlIGN1cnJlbnQgZ3JpZCxcbiAgICAvLyBlbmxhcmdlIGl0LlxuICAgIHdoaWxlICghdGhpcy50cnlQbGFjaW5nKHBpKSlcbiAgICB7XG4gICAgICAgIHRoaXMuc2l6ZVggKz0gMTA7XG4gICAgICAgIHRoaXMuc2l6ZVkgKz0gMTA7XG4gICAgICAgIHRoaXMubWFrZUdyaWQodGhpcy5zaXplWCwgdGhpcy5zaXplWSwgdGhpcy5jdXJtaW5vKTtcbiAgICB9XG5cbiAgICAvLyBtYXJrIHRoZSBwb3NpdGlvbnMgb2NjdXBpZWRcbiAgICBmb3IgKHZhciBrID0gMDsgayA8IHAubDsgaysrKVxuICAgIHtcbiAgICAgICAgdmFyIHh4ID0gcC5jb29yZFtrXS5nZXRYKCkgKyBwLng7XG4gICAgICAgIHZhciB5eSA9IHAuY29vcmRba10uZ2V0WSgpICsgcC55O1xuICAgICAgICB0aGlzLmdyaWRbeXldW3h4XSA9IDE7XG4gICAgfVxufTtcblxuLyoqXG4qIFRoaXMgbWV0aG9kIG1ha2VzIGEgcmFuZG9tIHBlcm11dGF0aW9uIG9mIHBvbHlvbWlubyBjZWxscyBhbmQgY2FsY3VsYXRlc1xuKiB0aGUgYm91bmRpbmcgcmVjdGFuZ2xlcyBvZiB0aGUgcG9seW9taW5vZXMuXG4qL1xuUG9seW9taW5vUGFja2luZy5wcm90b3R5cGUuUmFuZG9taXplTWlubyA9IGZ1bmN0aW9uIChwaSlcbntcbiAgICB2YXIgcCA9IHBvbHlvbWlub2VzW3BpXTtcbiAgICB2YXIgaTtcblxuICAgIC8vIG1ha2UgdGhlIHJhbmRvbSBwZXJtdXRhdGlvbi4gVGhlb3JldGljYWxseSBpdCBzcGVlZHMgdXAgdGhlXG4gICAgLy8gYWxnb3JpdGhtIGEgbGl0dGxlLlxuICAgIGZvciAoaSA9IDA7IGkgPCBwLmw7IGkrKylcbiAgICB7XG4gICAgICAgIHZhciBpMSA9IE1hdGgucmFuZG9tKCkgKiAocC5sIC0gaSkgKyBpO1xuICAgICAgICB2YXIgdG1wID0gcC5jb29yZFtpXTtcbiAgICAgICAgcC5jb29yZFtpXSA9IHAuY29vcmRbaTFdO1xuICAgICAgICBwLmNvb3JkW2kxXSA9IHRtcDtcbiAgICB9XG5cbiAgICAvLyBjYWxjdWxhdGUgdGhlIGJvdW5kaW5nIHJlY3RhbmdsZSBvZiB0aGUgcG9seW9taW5vXG4gICAgdGhpcy5yZWN0W3BpXSA9IG5ldyBSZWN0YW5nbGVEKCk7XG5cbiAgICB2YXIgbWluWCA9IEludGVnZXIuTUFYX1ZBTFVFO1xuICAgIHZhciBtaW5ZID0gSW50ZWdlci5NQVhfVkFMVUU7XG4gICAgdmFyIG1heFggPSBJbnRlZ2VyLk1JTl9WQUxVRTtcbiAgICB2YXIgbWF4WSA9IEludGVnZXIuTUlOX1ZBTFVFO1xuICAgIHAueCA9IHAueSA9IDA7XG5cbiAgICBmb3IgKGkgPSAwOyBpIDwgcC5sOyBpKyspXG4gICAge1xuICAgICAgICBpZiAocC5jb29yZFtpXS5nZXRYKCkgPCBtaW5YKVxuICAgICAgICB7XG4gICAgICAgICAgICBtaW5YID0gcC5jb29yZFtpXS5nZXRYKCk7XG4gICAgICAgIH1cbiAgICAgICAgaWYgKHAuY29vcmRbaV0uZ2V0WSgpIDwgbWluWSlcbiAgICAgICAge1xuICAgICAgICAgICAgbWluWSA9IHAuY29vcmRbaV0uZ2V0WSgpO1xuICAgICAgICB9XG4gICAgICAgIGlmIChwLmNvb3JkW2ldLmdldFgoKSA+IG1heFgpXG4gICAgICAgIHtcbiAgICAgICAgICAgIG1heFggPSBwLmNvb3JkW2ldLmdldFgoKTtcbiAgICAgICAgfVxuICAgICAgICBpZiAocC5jb29yZFtpXS5nZXRZKCkgPiBtYXhZKVxuICAgICAgICB7XG4gICAgICAgICAgICBtYXhZID0gcC5jb29yZFtpXS5nZXRZKCk7XG4gICAgICAgIH1cbiAgICB9XG5cbiAgICB0aGlzLnJlY3RbcGldLnggPSBtaW5YO1xuICAgIHRoaXMucmVjdFtwaV0ueSA9IG1pblk7XG4gICAgdGhpcy5yZWN0W3BpXS53aWR0aCA9IG1heFggLSBtaW5YO1xuICAgIHRoaXMucmVjdFtwaV0uaGVpZ2h0ID0gbWF4WSAtIG1pblk7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IFBvbHlvbWlub1BhY2tpbmc7IiwiZnVuY3Rpb24gUG9seW9taW5vUXVpY2tTb3J0KClcbntcbiAgICAvLyBUaGlzIHZhcmlhYmxlIHN0b3JlcyB0aGUgbnVtYmVyIG9mIGVsZW1lbnRzIHRvIGJlIHNvcnRlZFxuICAgIHRoaXMubiA9IDA7XG5cbiAgICAvLyBUaGlzIGFycmF5IHN0b3JlcyB0aGUgZWxlbWVudHMgdG8gYmUgc29ydGVkXG4gICAgdGhpcy5rZXkgPSBbXTtcblxuICAgIC8vIFRoaXMgYXJyYXkgaXMgdXNlZCBieSBzdGF0aWMgc29ydGVyIGFuZCByZXByZXNlbnRzIHRoZSBjdXJyZW50XG4gICAgLy8gb3JkZXIgb2YgdGhlIGlucHV0IGFycmF5XG4gICAgdGhpcy5pbmRleCA9IFtdO1xufVxuXG4vKipcbiogVGhpcyBtZXRob2QgaXMgYSBzdGF0aWMgc29ydGVyLlxuKiAgUGFyYW1ldGhlcnM6XG4qICAgICAgbiAtIGlzIHRoZSBudW1iZXIgb2YgZG91YmxlcyB0byBiZSBzb3J0ZWQ7XG4qICAgICAga2V5IC0gaXMgYXJyYXkgb2YgZG91YmxlcyB0byBiZSBzb3J0ZWQuIFRoZSBvcmRlciBvZiBlbGVtZW50c1xuKiAgICAgICAgICBpbiB0aGlzIGFycmF5IGlzIHByZXNlcnZlZDtcbiogICAgICBpbmRleCAtIGluIHRoaXMgYXJyYXkgdGhlIHNvcnRlZCBvcmRlciBvZiBlbGVtZW50cyBpc1xuKiAgICAgICAgICByZXR1cm5lZC4gaW5kZXhbaV0gPSBqIG1lYW5zIHRoYXQgaS10aCBncmVhdGVzdCBlbGVtZW50XG4qICAgICAgICAgIGlzIGotdGggZWxlbWVudCBvZiBhcnJheSBrZXkuXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5zb3J0ID0gZnVuY3Rpb24gKG4sIGtleSwgaW5kZXgpXG57XG4gICAvLyBzdG9yZSBhbGwgcGFyYW1ldGhlcnMgYXMgdGhlIGluc3RhbmNlIHZhcmlhYmxlc1xuICAgdGhpcy5uID0gbjtcbiAgIHRoaXMua2V5ID0ga2V5O1xuICAgdGhpcy5pbmRleCA9IGluZGV4O1xuXG4gICAvLyBtYWtlIHRoZSBpbml0aWFsIG9yZGVyIG9mIGVsZW1lbnRzXG4gICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMubjsgaSsrKVxuICAgICAgIHRoaXMuaW5kZXhbaV0gPSBpO1xuXG4gICAvLyBjYWxsIHRoZSByZWN1cnNpdmUgc29ydGluZyBtZXRob2Qgb24gdGhlIHdob2xlIGFycmF5XG4gICB0aGlzLnJlY1NvcnRTdGF0aWMoMCwgdGhpcy5uLTEpO1xuXG4gICAvLyBkbyB0aGUgZmluYWwgc29ydCB3aXRoIHRoZSBpbnNlcnRpb24gc29ydGluZyBhbGdvcml0aG1cbiAgIC8vIGJlY2F1c2UgdGhlIHJlY3Vyc2l2ZSBxdWlja3NvcnQgcm91dGluZSBkbyBub3Qgc29ydCBpbnRlcnZhbHNcbiAgIC8vIGxlc3MgdGhhbiAxMFxuICAgdGhpcy5pbnNlcnRpb25Tb3J0U3RhdGljKCk7XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgaXMgYSBzcGVjaWFsIHNvcnRlciB0aGF0IHNvcnRzIGFuIGludGVnZXIgYXJyYXkuXG4qIFRoZSBtZXRob2QgaXMgc3BlY2lhbCBiZWNhdXNlIHRoZSBlbGVtZW50cyBhIGFuZCBiIG9mIHRoZSBhcnJheVxuKiBhcmUgbm90IGNvbXBhcmVkIGFzIGludGVnZXIgdmFsdWVzLiBUaGUgZWxlbWVudHMgb2Ygb3RoZXIgYXJyYXlcbiogKGFycmF5IG9mIGtleXMpIHdpdGggaW5kaWNlcyBhIGFuZCBiIGFyZSBjb21wYXJlZCBpbnN0ZWFkLiBJZiB0d29cbiogZWxlbWVudHMgaGF2ZSBlcXVhbCBrZXlzLCB0aGVuIHRoZSByZWxhdGl2ZSBvcmRlcmluZyBvZiB0aGVzZVxuKiBlbGVtZW50cyBpcyBwcmVzZXJ2ZWQuXG4qICBQYXJhbWV0aGVyczpcbiogICAgICBuIC0gaXMgdGhlIG51bWJlciBvZiBlbGVtZW50cyBpbiB0aGUgaW50ZWdlciBhcnJheTtcbiogICAgICBpbmRleCAtIHRoaXMgaXMgdGhlIGludGVnZXIgYXJyYXkgdG8gYmUgc29ydGVkO1xuKiAgICAgIGtleSAtIHRoaXMgYXJyYXkgYWN0cyBhcyBrZXlzIGZvciBjb21wYXJpc29uIG9mIHR3byBpbnRlZ2VyXG4qICAgICAgICAgICAgICBlbGVtZW50cy4gSXQgaXMgYXNzdW1lZCB0aGF0IGVsZW1lbnRzIG9mIHRoZSBhcnJheVxuKiAgICAgICAgICAgICAgaW5kZXggYXJlIGluIHRoZSByYW5nZSBbMCAuLiBrZXkubGVuZ3RoLTFdLlxuKi9cblBvbHlvbWlub1F1aWNrU29ydC5wcm90b3R5cGUuaW5kZXhTb3J0ID0gZnVuY3Rpb24gKG4sIGluZGV4LCBrZXkpXG57XG4gICAvLyBXZSB3YW50IHRvIHJlZHVjZSB0aGUgcHJvYmxlbSB0byB0aGUgc2ltcGxlIHN0YXRpYyBzb3J0aW5nLlxuICAgLy8gRmlyc3QgYSBtYXBwaW5nIGYgZnJvbSB0aGUgc2V0IHswLC4uLG4tMX0gdG8gdGhlIHNldCBvZlxuICAgLy8gZWxlbWVudHMgaXMgZGVmaW5lZCBieSBmKGkpIC0+IGluZGV4W2ldLiBUaGVuIHRoZSBwcm9ibGVtXG4gICAvLyBzaW1wbHkgcmVkdWNlcyB0byB0aGUgc3RhdGljIHNvcnQgd2hlcmUgaS10aCBrZXkgdmFsdWUgaXNcbiAgIC8vIGtleVtmKGkpXS5cblxuICAgLy8gYWxsb2NhdGUgbWVtb3J5IGZvciB0aGUgdGVtcG9yYXJ5IGFycmF5cyBmb3IgdGhlIHN0YXRpYyBzb3J0XG4gICB2YXIgdGVtcEluZGV4ID0gW107XG4gICB2YXIgdGVtcEtleSA9IFtdO1xuXG4gICAvLyB0aGlzIGFycmF5IHdpbGwgc3RvcmUgdGhlIGNvcHkgb2YgYXJyYXkgaW5kZXhcbiAgIHZhciBvbGRJbmRleCA9IFtdO1xuXG4gICBmb3IgKHZhciBpID0gMDsgaSA8IG47IGkrKylcbiAgIHtcbiAgICAgICAvLyBzdG9yZSB0aGUga2V5IGZvciBzdGF0aWMgc29ydFxuICAgICAgIHRlbXBLZXlbaV0gPSBrZXlbaW5kZXhbaV1dO1xuXG4gICAgICAgLy8gcmVtZW1iZXIgdGhlIGktdGggdmFsdWUgb2YgdGhlIGluZGV4IGFycmF5XG4gICAgICAgb2xkSW5kZXhbaV0gPSBpbmRleFtpXTtcbiAgIH1cblxuICAgLy8gc29ydCB0aGUgYXV4aWxpYXJ5IGFycmF5c1xuICAgdGhpcy5zb3J0KG4sIHRlbXBLZXksIHRlbXBJbmRleCk7XG5cbiAgIC8vIGFuZCBwdXQgdGhlIHJlc3VsdCBiYWNrIGluIHRoZSBpbmRleCBhcnJheVxuXG4gICBmb3IgKHZhciBpID0gMDsgaSA8IG47IGkrKylcbiAgICAgICBpbmRleFtpXSA9IG9sZEluZGV4W3RlbXBJbmRleFtpXV07XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgaXMgYSBub24tc3RhdGljIHNvcnRlci5cbiogIFBhcmFtZXRoZXJzOlxuKiAgICAgIG4gLSBpcyB0aGUgbnVtYmVyIG9mIGRvdWJsZXMgdG8gYmUgc29ydGVkO1xuKiAgICAgIGtleSAtIGlzIGFycmF5IG9mIGRvdWJsZXMgdG8gYmUgc29ydGVkLlxuKi9cblBvbHlvbWlub1F1aWNrU29ydC5wcm90b3R5cGUuc29ydCA9IGZ1bmN0aW9uIChuLCBrZXkpXG57XG4gICAvLyBzdG9yZSBhbGwgcGFyYW1ldGhlcnMgYXMgdGhlIGluc3RhbmNlIHZhcmlhYmxlc1xuICAgdGhpcy5uID0gbjtcbiAgIHRoaXMua2V5ID0ga2V5O1xuXG4gICAvLyBjYWxsIHRoZSByZWN1cnNpdmUgc29ydGluZyBtZXRob2Qgb24gdGhlIHdob2xlIGFycmF5XG4gICB0aGlzLnJlY1NvcnROb25TdGF0aWMoMCwgdGhpcy5uLTEpO1xuXG4gICAvLyBkbyB0aGUgZmluYWwgc29ydCB3aXRoIHRoZSBpbnNlcnRpb24gc29ydGluZyBhbGdvcml0aG1cbiAgIC8vIGJlY2F1c2UgdGhlIHJlY3Vyc2l2ZSBxdWlja3NvcnQgcm91dGluZSBkbyBub3Qgc29ydCBpbnRlcnZhbHNcbiAgIC8vIGxlc3MgdGhhbiAxMFxuICAgdGhpcy5pbnNlcnRpb25Tb3J0Tm9uU3RhdGljKCk7XG59O1xuXG4vLy0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS1cbi8vIFNlY3Rpb246IE1ldGhvZHMgZm9yIHN0YXRpYyBzb3J0ZXJcbi8vLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuXG4vKipcbiAqIFRoaXMgbWV0aG9kIGRvIHRoZSByZWN1cnNpdmUgc29ydGluZyB3aXRoIHRoZSBxdWlja3NvcnRcbiAqIGFsZ29yaXRobSBmb3Igc3RhdGljIHNvcnRlci4gVGhlIGludGVydmFsIHRvIGJlIHNvcnRlZCBpc1xuICogc3BlY2lmaWVkIGJ5IHRoZSBwYXJhbWV0aGVycy5cbiAqL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5yZWNTb3J0U3RhdGljID0gZnVuY3Rpb24gKGxlZnQsIHJpZ2h0KVxue1xuICAgIC8vIHJldHVybiBpbW1lZGlhdGVseSBpZiB0aGUgaW50ZXJ2YWwgc3BlY2lmaWVkIGlzIHRvbyBzaG9ydFxuICAgIGlmIChsZWZ0KzEwID4gcmlnaHQpXG4gICAge1xuICAgICAgICByZXR1cm47XG4gICAgfVxuXG4gICAgLy8gY2hvb3NlIHRoZSBwaXZvdCBhbmQgc3dhcCBpdCB3aXRoIHRoZSBsYXN0IGVsZW1lbnQgb2ZcbiAgICAvLyB0aGUgZ2l2ZW4gaW50ZXJ2YWxcbiAgICB2YXIgcGl2b3QgPSB0aGlzLm1lZGlhbjNTdGF0aWMobGVmdCwgcmlnaHQpO1xuXG4gICAgLy8gaXRlcmF0ZSB0aHJvdWdoIHRoZSBnaXZlbiBpbnRlcnZhbCBmcm9tIGJvdGggZW5kc1xuICAgIC8vIHNpbXVsdGFuZW91c2x5XG4gICAgdmFyIGkgPSBsZWZ0O1xuICAgIHZhciBqID0gcmlnaHQtMTtcblxuICAgIGZvciAoOzspXG4gICAge1xuICAgICAgICAvLyBpbmNyZW1lbnQgaSB3aGlsZSBpLXRoIGVsZW1lbnQgaXMgbGVzcyB0aGFuIHBpdm90XG4gICAgICAgIGRvXG4gICAgICAgIHtcbiAgICAgICAgICAgIGkrKztcbiAgICAgICAgfVxuICAgICAgICB3aGlsZSAodGhpcy5jbXAodGhpcy5pbmRleFtpXSwgcGl2b3QpID09PSAtMSk7XG5cbiAgICAgICAgLy8gZGVjcmVtZW50IGogd2hpbGUgai10aCBlbGVtZW50IGlzIGdyZWF0ZXIgdGhhbiBwaXZvdFxuICAgICAgICBkb1xuICAgICAgICB7XG4gICAgICAgICAgICBqLS07XG4gICAgICAgIH1cbiAgICAgICAgd2hpbGUgKHRoaXMuY21wKHRoaXMuaW5kZXhbal0sIHBpdm90KSA9PT0gMSk7XG5cbiAgICAgICAgLy8gaWYgaSBhbmQgaiBjcm9zcyB0aGVuIHdlIGFyZSBkb25lLCBvdGhlcndpc2Ugc3dhcCBpLXRoXG4gICAgICAgIC8vIGFuZCBqLXRoIGVsZW1lbnRzXG5cbiAgICAgICAgaWYgKGkgPCBqKVxuICAgICAgICB7XG4gICAgICAgICAgICB0aGlzLnN3YXBTdGF0aWMoaSxqKTtcbiAgICAgICAgfVxuICAgICAgICBlbHNlXG4gICAgICAgIHtcbiAgICAgICAgICAgIGJyZWFrO1xuICAgICAgICB9XG4gICAgfVxuXG4gICAgLy8gcGxhY2UgdGhlIHBpdm90IGluIHRoZSByaWdodCBwbGFjZVxuICAgIHRoaXMuc3dhcFN0YXRpYyhpLCByaWdodC0xKTtcblxuICAgIC8vIHJlY3Vyc2l2ZWx5IHNvcnQgYm90aCBwYXJ0cyB0byB0aGUgbGVmdCBhbmQgdG8gdGhlIHJpZ2h0XG4gICAgLy8gZnJvbSB0aGUgcGl2b3RcbiAgICB0aGlzLnJlY1NvcnRTdGF0aWMobGVmdCwgaS0xKTtcbiAgICB0aGlzLnJlY1NvcnRTdGF0aWMoaSsxLCByaWdodCk7XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2QgY2hvb3NlcyB0aGUgbWlkZGxlIGVsZW1lbnQgb2YgbGVmdCwgcmlnaHQgYW5kIGNlbnRlclxuKiBlbGVtZW50IG9mIGdpdmVuIGludGVydmFsIGFuZCByZXR1cm5zIHRoZSBpbmRleCBvZiB0aGlzIGVsZW1lbnQuXG4qIFRoaXMgbWV0aG9kIGlzIHVzZWQgYnkgc3RhdGljIHNvcnRlci5cbiovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLm1lZGlhbjNTdGF0aWMgPSBmdW5jdGlvbiAobGVmdCwgcmlnaHQpXG57XG4gICAvLyBjYWxjdWxhdGVzIHRoZSBjZW50ZXIgb2YgdGhlIGdpdmVuIGludGVydmFsXG4gICB2YXIgY2VudGVyID0gKGxlZnQrcmlnaHQpLzI7XG5cbiAgIC8vIGlmIGxlZnQgZWxlbWVudCBpcyBncmVhdGVyIHRoYW4gY2VudGVyIGVsZW1lbnQsIHN3YXAgdGhlbVxuICAgaWYgKHRoaXMuY21wKHRoaXMuaW5kZXhbbGVmdF0sIHRoaXMuaW5kZXhbY2VudGVyXSkgPT09IDEpXG4gICB7XG4gICAgICAgdGhpcy5zd2FwU3RhdGljKGxlZnQsY2VudGVyKTtcbiAgIH1cbiAgIFxuICAgLy8gaWYgbGVmdCBlbGVtZW50IGlzIGdyZWF0ZXIgdGhhbiByaWdodCBlbGVtZW50LCBzd2FwIHRoZW1cbiAgIGlmICh0aGlzLmNtcCh0aGlzLmluZGV4W2xlZnRdLCB0aGlzLmluZGV4W3JpZ2h0XSkgPT09IDEpXG4gICB7XG4gICAgICAgdGhpcy5zd2FwU3RhdGljKGxlZnQscmlnaHQpO1xuICAgfVxuICAgXG4gICAvLyBpZiBjZW50ZXIgZWxlbWVudCBpcyBncmVhdGVyIHRoYW4gcmlnaHQgZWxlbWVudCwgc3dhcCB0aGVtXG4gICBpZiAodGhpcy5jbXAodGhpcy5pbmRleFtjZW50ZXJdLCB0aGlzLmluZGV4W3JpZ2h0XSkgPT09IDEpXG4gICB7XG4gICAgICAgdGhpcy5zd2FwU3RhdGljKGNlbnRlcixyaWdodCk7XG4gICB9XG4gICBcbiAgIC8vIG5vdyB0aGUgY2VudGVyIGVsZW1lbnQgaXMgbGVzcyBvciBlcXVhbCB0aGFuIHJpZ2h0IGVsZW1lbnRcbiAgIC8vIGFuZCBncmVhdGVyIG9yIGVxdWFsIHRoYW4gbGVmdCBlbGVtZW50XG5cbiAgIC8vIG1vdmUgdGhlIGNlbnRlciBlbGVtZW50IHRvIHRoZSByaWdodCBzaWRlIGJ5IHN3YXBpbmcgaXQgd2l0aFxuICAgLy8gdGhlIG9uZSBiZWZvcmUgdGhlIHJpZ2h0IGVsZW1lbnRcbiAgIHRoaXMuc3dhcFN0YXRpYyhjZW50ZXIsIHJpZ2h0LTEpO1xuXG4gICAvLyByZXR1cm4gdGhlIHBpdm90J3MgaW5kZXhcbiAgIHJldHVybiB0aGlzLmluZGV4W3JpZ2h0LTFdO1xufTtcblxuLyoqXG4qIFRoaXMgbWV0aG9kIGNvbXBhcmVzIHR3byBlbGVtZW50cyBzcGVjaWZpZWQgd2l0aCB0aGVpciBpbmRpY2VzLlxuKiBJdCByZXR1cm5zIC0xLCBpZiBpLXRoIGVsZW1lbnQgaXMgbGVzcyB0aGFuIGotdGggZWxlbWVudCwgYW5kIDEsXG4qIGlmIGotdGggZWxlbWVudCBpcyBsZXNzIHRoYW4gaS10aCBlbGVtZW50LiBJZiBib3RoIGVsZW1lbnRzIGFyZVxuKiBlcXVhbCB0aGVuIHRoZWlyIGluZGljZXMgaSBhbmQgaiBhcmUgY29tcGFyZWQuIFRoaXMgbWV0aG9kIGlzXG4qIHVzZWQgb25seSBieSBzdGF0aWMgc29ydGVyLlxuKi9cblBvbHlvbWlub1F1aWNrU29ydC5wcm90b3R5cGUuY21wID0gZnVuY3Rpb24gKGksIGopXG57XG4gICBpZiAodGhpcy5rZXlbaV0gPCB0aGlzLmtleVtqXSlcbiAgIHtcbiAgICAgICByZXR1cm4gLTE7XG4gICB9XG4gICBcbiAgIGlmICh0aGlzLmtleVtpXSA+IHRoaXMua2V5W2pdKVxuICAge1xuICAgICAgIHJldHVybiAxO1xuICAgfVxuICAgXG4gICBpZiAoaSA8IGopXG4gICB7XG4gICAgICAgcmV0dXJuIC0xO1xuICAgfVxuICAgXG4gICBpZiAoaSA+IGopXG4gICB7XG4gICAgICAgcmV0dXJuIDE7XG4gICB9XG4gICBcbiAgIHJldHVybiAwO1xufTtcblxuLyoqXG4qIFRoaXMgbWV0aG9kIHN3YXBzIHRoZSBlbGVtZW50cyBieSBzd2FwcGluZyB0aGVpciBpbmRpY2VzLiBVc2VkXG4qIG9ubHkgYnkgc3RhdGljIHNvcnRlci5cbiovXG5Qb2x5b21pbm9RdWlja1NvcnQucHJvdG90eXBlLnN3YXBTdGF0aWMgPSBmdW5jdGlvbiAoaSwgailcbntcbiAgIHZhciB0ZW1wID0gdGhpcy5pbmRleFtpXTtcbiAgIHRoaXMuaW5kZXhbaV0gPSB0aGlzLmluZGV4W2pdO1xuICAgdGhpcy5pbmRleFtqXSA9IHRlbXA7XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2Qgc29ydHMgdGhlIGFycmF5IHdpdGggdGhlIGluc2VydGlvbiBzb3J0IGFsZ29yaXRobS5cbiogVGhpcyBtZXRob2QgaXMgdXNlZCBvbmx5IGJ5IHN0YXRpYyBzb3J0ZWQgYW5kIHRoZXJlZm9yZSBpdCB3b3Jrc1xuKiB3aXRoIGFycmF5IG9mIGluZGljZXMgaW5zdGVhZCBvZiBpbnB1dCBhcnJheSBpdHNlbGYuXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5pbnNlcnRpb25Tb3J0U3RhdGljID0gZnVuY3Rpb24gKClcbntcbiAgIC8vIGl0ZXJhdGUgdGhyb3VnaCBhbGwgZWxlbWVudHNcbiAgIGZvciAodmFyIGkgPSAxOyBpIDwgdGhpcy5uOyBpKyspXG4gICB7XG4gICAgICAgLy8gc3RvcmVzIHRoZSBpbmRleCBvZiBjdXJyZW50IGVsZW1lbnRcbiAgICAgICB2YXIgdGVtcCA9IHRoaXMuaW5kZXhbaV07XG5cbiAgICAgICAvLyBzZWFyY2ggdGhlIHJpZ2h0IHNwb3QgZm9yIGN1cnJlbnQgZWxlbWVudCBieSBpdGVyYXRpbmdcbiAgICAgICAvLyBiYWNrd2FyZHMgYW5kIHB1c2hpbmcgZ3JlYXRlciBlbGVtZW50cyBvbmUgcGxhY2UgdXBcbiAgICAgICB2YXIgaiA9IGk7XG5cbiAgICAgICB3aGlsZSAoaiA+PSAxICYmIHRoaXMuY21wKHRoaXMuaW5kZXhbai0xXSwgdGVtcCkgPT09IDEpXG4gICAgICAge1xuICAgICAgICAgICAvLyBqLXRoIGVsZW1lbnQgaXMgZ3JlYXRlciB0aGFuIGN1cnJlbnQgc28gbW92ZSBpdCB1cFxuICAgICAgICAgICB0aGlzLmluZGV4W2pdID0gdGhpcy5pbmRleFtqLTFdO1xuICAgICAgICAgICBqLS07XG4gICAgICAgfVxuXG4gICAgICAgLy8gd2UgZm91bmQgdGhlIG5ldyBob21lIGZvciBjdXJyZW50IGVsZW1lbnQgc28gc3RvcmUgaXRcbiAgICAgICB0aGlzLmluZGV4W2pdID0gdGVtcDtcbiAgIH1cbn07XG5cbi8vLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuLy8gU2VjdGlvbjogTWV0aG9kcyBmb3Igbm9uLXN0YXRpYyBzb3J0ZXJcbi8vLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLS0tLVxuXG4vKipcbiAqIFRoaXMgbWV0aG9kIGRvIHRoZSByZWN1cnNpdmUgc29ydGluZyB3aXRoIHRoZSBxdWlja3NvcnRcbiAqIGFsZ29yaXRobSBmb3Igbm9uLXN0YXRpYyBzb3J0ZXIuIFRoZSBpbnRlcnZhbCB0byBiZSBzb3J0ZWQgaXNcbiAqIHNwZWNpZmllZCBieSB0aGUgcGFyYW1ldGhlcnMuXG4gKi9cblBvbHlvbWlub1F1aWNrU29ydC5wcm90b3R5cGUucmVjU29ydE5vblN0YXRpYyA9IGZ1bmN0aW9uIChsZWZ0LCByaWdodClcbntcbiAgICAvLyByZXR1cm4gaW1tZWRpYXRlbHkgaWYgdGhlIGludGVydmFsIHNwZWNpZmllZCBpcyB0b28gc2hvcnRcbiAgICBpZiAobGVmdCsxMCA+IHJpZ2h0KVxuICAgIHtcbiAgICAgICAgcmV0dXJuO1xuICAgIH1cbiAgICBcbiAgICAvLyBjaG9vc2UgdGhlIHBpdm90IGFuZCBzd2FwIGl0IHdpdGggdGhlIGxhc3QgZWxlbWVudCBvZlxuICAgIC8vIHRoZSBnaXZlbiBpbnRlcnZhbFxuICAgIHZhciBwaXZvdCA9IHRoaXMubWVkaWFuM05vblN0YXRpYyhsZWZ0LCByaWdodCk7XG5cbiAgICAvLyBpdGVyYXRlIHRocm91Z2ggdGhlIGdpdmVuIGludGVydmFsIGZyb20gYm90aCBlbmRzXG4gICAgLy8gc2ltdWx0YW5lb3VzbHlcbiAgICB2YXIgaSA9IGxlZnQ7XG4gICAgdmFyIGogPSByaWdodC0xO1xuXG4gICAgZm9yICg7OylcbiAgICB7XG4gICAgICAgIC8vIGluY3JlbWVudCBpIHdoaWxlIGktdGggZWxlbWVudCBpcyBsZXNzIHRoYW4gcGl2b3RcbiAgICAgICAgZG9cbiAgICAgICAge1xuICAgICAgICAgICAgaSsrO1xuICAgICAgICB9XG4gICAgICAgIHdoaWxlICh0aGlzLmtleVtpXSA8IHBpdm90KTtcblxuICAgICAgICAvLyBkZWNyZW1lbnQgaiB3aGlsZSBqLXRoIGVsZW1lbnQgaXMgZ3JlYXRlciB0aGFuIHBpdm90XG4gICAgICAgIGRvXG4gICAgICAgIHtcbiAgICAgICAgICAgIGotLTtcbiAgICAgICAgfVxuICAgICAgICB3aGlsZSAodGhpcy5rZXlbal0gPiBwaXZvdCk7XG5cbiAgICAgICAgLy8gaWYgaSBhbmQgaiBjcm9zcyB0aGVuIHdlIGFyZSBkb25lLCBvdGhlcndpc2Ugc3dhcCBpLXRoXG4gICAgICAgIC8vIGFuZCBqLXRoIGVsZW1lbnRzXG4gICAgICAgIGlmIChpIDwgailcbiAgICAgICAge1xuICAgICAgICAgICAgdGhpcy5zd2FwTm9uU3RhdGljKGksIGopO1xuICAgICAgICB9XG4gICAgICAgIGVsc2VcbiAgICAgICAge1xuICAgICAgICAgICAgYnJlYWs7XG4gICAgICAgIH1cbiAgICB9XG5cbiAgICAvLyBwbGFjZSB0aGUgcGl2b3QgaW4gdGhlIHJpZ2h0IHBsYWNlXG4gICAgdGhpcy5zd2FwTm9uU3RhdGljKGksIHJpZ2h0LTEpO1xuXG4gICAgLy8gcmVjdXJzaXZlbHkgc29ydCBib3RoIHBhcnRzIHRvIHRoZSBsZWZ0IGFuZCB0byB0aGUgcmlnaHRcbiAgICAvLyBmcm9tIHRoZSBwaXZvdFxuICAgIHRoaXMucmVjU29ydE5vblN0YXRpYyhsZWZ0LCBpLTEpO1xuICAgIHRoaXMucmVjU29ydE5vblN0YXRpYyhpKzEsIHJpZ2h0KTtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCByZXR1cm5zIHRoZSBtaWRkbGUgZWxlbWVudCBvZiBsZWZ0LCByaWdodCBhbmQgY2VudGVyXG4qIGVsZW1lbnQuIFRoaXMgbWV0aG9kIGlzIHVzZWQgYnkgbm9uLXN0YXRpYyBzb3J0ZXIuXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5tZWRpYW4zTm9uU3RhdGljID0gZnVuY3Rpb24gKGxlZnQsIHJpZ2h0KVxue1xuICAgIC8vIGNhbGN1bGF0ZXMgdGhlIGNlbnRlciBvZiB0aGUgZ2l2ZW4gaW50ZXJ2YWxcbiAgICB2YXIgY2VudGVyID0gKGxlZnQrcmlnaHQpLzI7XG5cbiAgICAvLyBpZiBsZWZ0IGVsZW1lbnQgaXMgZ3JlYXRlciB0aGFuIGNlbnRlciBlbGVtZW50LCBzd2FwIHRoZW1cbiAgICBpZiAodGhpcy5rZXlbbGVmdF0gPiB0aGlzLmtleVtjZW50ZXJdKVxuICAgIHtcbiAgICAgICAgdGhpcy5zd2FwTm9uU3RhdGljKGxlZnQsIGNlbnRlcik7XG4gICAgfVxuICAgXG4gICAgLy8gaWYgbGVmdCBlbGVtZW50IGlzIGdyZWF0ZXIgdGhhbiByaWdodCBlbGVtZW50LCBzd2FwIHRoZW1cbiAgICBpZiAodGhpcy5rZXlbbGVmdF0gPiB0aGlzLmtleVtyaWdodF0pXG4gICAge1xuICAgICAgICB0aGlzLnN3YXBOb25TdGF0aWMobGVmdCwgcmlnaHQpO1xuICAgIH1cbiAgICBcbiAgICAvLyBpZiBjZW50ZXIgZWxlbWVudCBpcyBncmVhdGVyIHRoYW4gcmlnaHQgZWxlbWVudCwgc3dhcCB0aGVtXG4gICAgaWYgKHRoaXMua2V5W2NlbnRlcl0gPiB0aGlzLmtleVtyaWdodF0pXG4gICAge1xuICAgICAgICB0aGlzLnN3YXBOb25TdGF0aWMoY2VudGVyLCByaWdodCk7XG4gICAgfVxuICAgIFxuICAgIC8vIG5vdyB0aGUgY2VudGVyIGVsZW1lbnQgaXMgbGVzcyBvciBlcXVhbCB0aGFuIHJpZ2h0IGVsZW1lbnRcbiAgICAvLyBhbmQgZ3JlYXRlciBvciBlcXVhbCB0aGFuIGxlZnQgZWxlbWVudFxuXG4gICAgLy8gbW92ZSB0aGUgY2VudGVyIGVsZW1lbnQgdG8gdGhlIHJpZ2h0IHNpZGUgYnkgc3dhcGluZyBpdCB3aXRoXG4gICAgLy8gdGhlIG9uZSBiZWZvcmUgdGhlIHJpZ2h0IGVsZW1lbnRcbiAgIHRoaXMuc3dhcE5vblN0YXRpYyhjZW50ZXIsIHJpZ2h0LTEpO1xuXG4gICAvLyByZXR1cm4gdGhlIHBpdm90XG4gICByZXR1cm4gdGhpcy5rZXlbcmlnaHQtMV07XG59O1xuXG4vKipcbiogVGhpcyBtZXRob2Qgc3dhcHMgdGhlIGVsZW1lbnRzLiBVc2VkIG9ubHkgYnkgbm9uLXN0YXRpYyBzb3J0ZXIuXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5zd2FwTm9uU3RhdGljID0gZnVuY3Rpb24gKGksIGopXG57XG4gICB2YXIgdGVtcCA9IHRoaXMua2V5W2ldO1xuICAgdGhpcy5rZXlbaV0gPSB0aGlzLmtleVtqXTtcbiAgIHRoaXMua2V5W2pdID0gdGVtcDtcbn07XG5cbi8qKlxuKiBUaGlzIG1ldGhvZCBzb3J0cyB0aGUgYXJyYXkgd2l0aCB0aGUgaW5zZXJ0aW9uIHNvcnQgYWxnb3JpdGhtLlxuKiBUaGlzIG1ldGhvZCBpcyB1c2VkIG9ubHkgYnkgbm9uLXN0YXRpYyBzb3J0ZWQuXG4qL1xuUG9seW9taW5vUXVpY2tTb3J0LnByb3RvdHlwZS5pbnNlcnRpb25Tb3J0Tm9uU3RhdGljID0gZnVuY3Rpb24gKClcbntcbiAgICAvLyBpdGVyYXRlIHRocm91Z2ggYWxsIGVsZW1lbnRzXG4gICAgZm9yICh2YXIgaSA9IDE7IGkgPCB0aGlzLm47IGkrKylcbiAgICB7XG4gICAgICAgIC8vIHN0b3JlcyB0aGUgY3VycmVudCBlbGVtZW50XG4gICAgICAgIHZhciB0ZW1wID0gdGhpcy5rZXlbaV07XG5cbiAgICAgICAgLy8gc2VhcmNoIHRoZSByaWdodCBzcG90IGZvciBjdXJyZW50IGVsZW1lbnQgYnkgaXRlcmF0aW5nXG4gICAgICAgIC8vIGJhY2t3YXJkcyBhbmQgcHVzaGluZyBncmVhdGVyIGVsZW1lbnRzIG9uZSBwbGFjZSB1cFxuICAgICAgICB2YXIgaiA9IGk7XG5cbiAgICAgICAgd2hpbGUgKGogPj0gMSAmJiB0aGlzLmtleVtqLTFdID4gdGVtcClcbiAgICAgICAge1xuICAgICAgICAgICAgLy8gai0xLXRoIGVsZW1lbnQgaXMgZ3JlYXRlciB0aGFuIGN1cnJlbnQgc28gbW92ZSBpdCB1cFxuICAgICAgICAgICAgdGhpcy5rZXlbal0gPSB0aGlzLmtleVtqLTFdO1xuICAgICAgICAgICAgai0tO1xuICAgICAgICB9XG4gICAgICAgXG4gICAgICAgIC8vIHdlIGZvdW5kIHRoZSBuZXcgaG9tZSBmb3IgY3VycmVudCBlbGVtZW50IHNvIHN0b3JlIGl0XG4gICAgICAgIHRoaXMua2V5W2pdID0gdGVtcDtcbiAgICB9XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IFBvbHlvbWlub1F1aWNrU29ydDsiLCJmdW5jdGlvbiBSYW5kb21TZWVkKCkge1xufVxuUmFuZG9tU2VlZC5zZWVkID0gMTtcblJhbmRvbVNlZWQueCA9IDA7XG5cblJhbmRvbVNlZWQubmV4dERvdWJsZSA9IGZ1bmN0aW9uICgpIHtcbiAgUmFuZG9tU2VlZC54ID0gTWF0aC5zaW4oUmFuZG9tU2VlZC5zZWVkKyspICogMTAwMDA7XG4gIHJldHVybiBSYW5kb21TZWVkLnggLSBNYXRoLmZsb29yKFJhbmRvbVNlZWQueCk7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IFJhbmRvbVNlZWQ7XG4iLCJ2YXIgUmFuZG9tU2VlZCA9IHJlcXVpcmUoJy4vUmFuZG9tU2VlZCcpO1xyXG52YXIgUG9seW9taW5vID0gcmVxdWlyZSgnLi9Qb2x5b21pbm8nKTtcclxudmFyIFBvaW50ID0gcmVxdWlyZSgnLi9Qb2ludCcpO1xyXG5cclxuZnVuY3Rpb24gUmVjdFByb2MoKSB7XHJcbn1cclxuXHJcblJlY3RQcm9jLkFzcGVjdFJhdGlvID0gKDEuMCAvIDEuMCk7Ly8geXNpemUveHNpemVcclxuXHJcblJlY3RQcm9jLlBsYWNlUmFuZG9tbHkgPSBmdW5jdGlvbiAock4sIHJYMSwgclkxLCByTCwgckgpXHJcbntcclxuICAgIHZhciBpbmRleEFycmF5ID0gW107XHJcblxyXG4gICAgdmFyIHN1bUwgPSAwO1xyXG4gICAgdmFyIHN1bUggPSAwO1xyXG5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIHtcclxuICAgICAgICBzdW1MICs9IHJMW2ldO1xyXG4gICAgICAgIHN1bUggKz0gckhbaV07XHJcbiAgICAgICAgaW5kZXhBcnJheVtpXSA9IGk7XHJcbiAgICB9XHJcbiAgICBcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgYSA9IFJhbmRvbVNlZWQubmV4dERvdWJsZSgvKiBUT0RPOiByTiAqLyk7XHJcbiAgICAgICAgdmFyIHRtcCA9IGluZGV4QXJyYXlbaV07XHJcbiAgICAgICAgaW5kZXhBcnJheVtpXSA9IGluZGV4QXJyYXlbYV07XHJcbiAgICAgICAgaW5kZXhBcnJheVthXSA9IHRtcDtcclxuICAgIH1cclxuXHJcbiAgICBzdW1MIC89IHJOO1xyXG4gICAgc3VtSCAvPSByTjtcclxuICAgIHZhciBudW1Sb3dzID0gKGludCkgKE1hdGguc3FydChyTikgKyAwLjQ5OTkpO1xyXG5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIHtcclxuICAgICAgICByWDFbaW5kZXhBcnJheVtpXV0gPSAoaSAvIG51bVJvd3MpICogc3VtTDtcclxuICAgICAgICByWTFbaW5kZXhBcnJheVtpXV0gPSAoaSAlIG51bVJvd3MpICogc3VtSDtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIHBhY2tzIHJlY3RhbmdsZXMgdXNpbmcgcG9seW9taW5vIHBhY2tpbmcgYWxnb3JpdGhtLlxyXG4qIFxyXG4qIEByZXR1cm5cclxuKi9cclxuUmVjdFByb2MucGFja1JlY3RhbmdsZXNNaW5vICA9IGZ1bmN0aW9uIChidWZmZXIsIHJOLCByZWN0YW5nbGVzKVxyXG57XHJcbiAgICAvLyBtYWtlIHRoZSBpbnRlcm1lZGlhdGUgZGF0YSBzdHJ1Y3R1cmVcclxuICAgIHZhciByWDEgPSBbXTtcclxuICAgIHZhciByWTEgPSBbXTtcclxuICAgIHZhciByVyA9IFtdO1xyXG4gICAgdmFyIHJIID0gW107XHJcblxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCByTjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHJYMVtpXSA9IHJlY3RhbmdsZXNbaV0uZ2V0Q2VudGVyWCgpO1xyXG4gICAgICAgIHJZMVtpXSA9IHJlY3RhbmdsZXNbaV0uZ2V0Q2VudGVyWSgpO1xyXG4gICAgICAgIHJXW2ldID0gcmVjdGFuZ2xlc1tpXS5nZXRXaWR0aCgpO1xyXG4gICAgICAgIHJIW2ldID0gcmVjdGFuZ2xlc1tpXS5nZXRIZWlnaHQoKTtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJOOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgclgxW2ldIC09IHJXW2ldIC8gMjtcclxuICAgICAgICByWTFbaV0gLT0gckhbaV0gLyAyO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRvIHRoZSBwYWNraW5nXHJcbiAgICBSZWN0UHJvYy5wYWNrUmVjdGFuZ2xlc01pbm8oYnVmZmVyLCByTiwgclgxLCByVywgclkxLCBySCwgcmVjdGFuZ2xlcyk7XHJcblxyXG4gICAgLy8gdHJhbnNmZXIgYmFjayB0aGUgcmVzdWx0c1xyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCByTjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHJYMVtpXSArPSByV1tpXSAvIDI7XHJcbiAgICAgICAgclkxW2ldICs9IHJIW2ldIC8gMjtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJOOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgcmVjdGFuZ2xlc1tpXS5zZXRDZW50ZXIoclgxW2ldLCByWTFbaV0pO1xyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgcGFja3MgcmVjdGFuZ2xlcyB1c2luZyBwb2x5b21pbm8gcGFja2luZyBhbGdvcml0aG0uXHJcbiovXHJcblxyXG5SZWN0UHJvYy5wYWNrUmVjdGFuZ2xlc01pbm8gPSBmdW5jdGlvbiAoYnVmZmVyLCByTiwgclgsIHJXLCByWSwgckgsIHJlY3RhbmdsZXMpXHJcbntcclxuICAgIGlmIChyTiA9PSAwKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybjtcclxuICAgIH1cclxuXHJcbiAgICB2YXIgc3RlcFggPSA1O1xyXG4gICAgdmFyIHN0ZXBZID0gNTtcclxuXHJcbiAgICAvLyBkeW5hbWljYWxseSBjYWxjdWxhdGUgdGhlIGdyaWQgc3RlcFxyXG4gICAgLy8gZG91YmxlIGFyZWEgPSAwO1xyXG4gICAgLy9cclxuICAgIC8vIGZvciAoaW50IGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIC8vIHtcclxuICAgIC8vIC8vIHN0ZXBYKz1yTFtpXStkZWx0YTtcclxuICAgIC8vIC8vIHN0ZXBZKz1ySFtpXStkZWx0YTtcclxuICAgIC8vIGFyZWEgKz0gKHJXW2ldICsgZGVsdGEpICogKHJIW2ldICsgZGVsdGEpO1xyXG4gICAgLy8gfVxyXG4gICAgLy9cclxuICAgIC8vIGRvdWJsZSBzdGVwWCA9IE1hdGguc3FydChhcmVhIC8gKHJOICogMTYpKTtcclxuICAgIC8vIC8vIChzdGVwWCtzdGVwWSkvKHJOKjgpO1xyXG4gICAgLy9cclxuXHJcbiAgICAvLyBhZGp1c3QgcmVzcGVjdGluZyB0aGUgYXNwZWN0IHJhdGlvXHJcblxyXG4gICAgdmFyIGZzdGVwID0gMiAvICgxICsgUmVjdFByb2MuQXNwZWN0UmF0aW8pO1xyXG4gICAgc3RlcFkgPSBzdGVwWCAqIFJlY3RQcm9jLkFzcGVjdFJhdGlvICogZnN0ZXA7XHJcbiAgICBzdGVwWCAqPSBmc3RlcDtcclxuXHJcbiAgICAvLyBtYWtlIHRoZSBwb2x5b21pbm8gcmVwcmVzZW50YXRpb25cclxuICAgIHZhciBtaW5vcyA9IFtdO1xyXG5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgck47IGkrKylcclxuICAgIHtcclxuICAgICAgICAvLyBzaXplIG9mIHRoZSByZWN0YW5nbGUgaW4gZ3JpZCB1bml0c1xyXG4gICAgICAgIHZhciBXID0gTWF0aC5jZWlsKChyV1tpXSArIGJ1ZmZlcikgLyBzdGVwWCk7XHJcbiAgICAgICAgdmFyIEggPSBNYXRoLmNlaWwoKHJIW2ldICsgYnVmZmVyKSAvIHN0ZXBZKTtcclxuXHJcbiAgICAgICAgbWlub3NbaV0gPSBuZXcgUG9seW9taW5vKCk7XHJcbiAgICAgICAgbWlub3NbaV0uY29vcmQgPSBbXTtcclxuXHJcbiAgICAgICAgLy8gY3JlYXRlIHRoZSBwb2x5b21pbm8gY2VsbHNcclxuICAgICAgICB2YXIgY250ID0gMDtcclxuICAgICAgICBmb3IgKHZhciB5ID0gMDsgeSA8IEg7IHkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGZvciAodmFyIHggPSAwOyB4IDwgVzsgeCsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBtaW5vc1tpXS5jb29yZFtjbnRdID0gbmV3IFBvaW50KCk7XHJcbiAgICAgICAgICAgICAgICBtaW5vc1tpXS5jb29yZFtjbnRdLnggPSB4O1xyXG4gICAgICAgICAgICAgICAgbWlub3NbaV0uY29vcmRbY250KytdLnkgPSB5O1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIG1pbm9zW2ldLmwgPSBjbnQ7XHJcbiAgICAgICAgbWlub3NbaV0ubGFiZWwgPSByZWN0YW5nbGVzW2ldLmxhYmVsO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGRvIHRoZSBwYWNraW5nXHJcbiAgICB2YXIgcGFja2VyID0gbmV3IFBvbHlvbWlub1BhY2tpbmcoKTtcclxuICAgIHBhY2tlci5wYWNrKG1pbm9zLCByTik7XHJcblxyXG4gICAgLy8gZ2V0IHRoZSByZXN1bHRzXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJOOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgclhbaV0gPSBtaW5vc1tpXS54ICogc3RlcFg7XHJcbiAgICAgICAgcllbaV0gPSBtaW5vc1tpXS55ICogc3RlcFk7XHJcbiAgICB9XHJcbn07XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IFJlY3RQcm9jOyIsImZ1bmN0aW9uIFJlY3RhbmdsZUQoeCwgeSwgd2lkdGgsIGhlaWdodCkge1xuICB0aGlzLnggPSAwO1xuICB0aGlzLnkgPSAwO1xuICB0aGlzLndpZHRoID0gMDtcbiAgdGhpcy5oZWlnaHQgPSAwO1xuXG4gIGlmICh4ICE9IG51bGwgJiYgeSAhPSBudWxsICYmIHdpZHRoICE9IG51bGwgJiYgaGVpZ2h0ICE9IG51bGwpIHtcbiAgICB0aGlzLnggPSB4O1xuICAgIHRoaXMueSA9IHk7XG4gICAgdGhpcy53aWR0aCA9IHdpZHRoO1xuICAgIHRoaXMuaGVpZ2h0ID0gaGVpZ2h0O1xuICB9XG59XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmdldFggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy54O1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuc2V0WCA9IGZ1bmN0aW9uICh4KVxue1xuICB0aGlzLnggPSB4O1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0WSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnk7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5zZXRZID0gZnVuY3Rpb24gKHkpXG57XG4gIHRoaXMueSA9IHk7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRXaWR0aCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLndpZHRoO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuc2V0V2lkdGggPSBmdW5jdGlvbiAod2lkdGgpXG57XG4gIHRoaXMud2lkdGggPSB3aWR0aDtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmdldEhlaWdodCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmhlaWdodDtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLnNldEhlaWdodCA9IGZ1bmN0aW9uIChoZWlnaHQpXG57XG4gIHRoaXMuaGVpZ2h0ID0gaGVpZ2h0O1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0UmlnaHQgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy54ICsgdGhpcy53aWR0aDtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmdldEJvdHRvbSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnkgKyB0aGlzLmhlaWdodDtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmludGVyc2VjdHMgPSBmdW5jdGlvbiAoYSlcbntcbiAgaWYgKHRoaXMuZ2V0UmlnaHQoKSA8IGEueClcbiAge1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuXG4gIGlmICh0aGlzLmdldEJvdHRvbSgpIDwgYS55KVxuICB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgaWYgKGEuZ2V0UmlnaHQoKSA8IHRoaXMueClcbiAge1xuICAgIHJldHVybiBmYWxzZTtcbiAgfVxuXG4gIGlmIChhLmdldEJvdHRvbSgpIDwgdGhpcy55KVxuICB7XG4gICAgcmV0dXJuIGZhbHNlO1xuICB9XG5cbiAgcmV0dXJuIHRydWU7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRDZW50ZXJYID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMueCArIHRoaXMud2lkdGggLyAyO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0TWluWCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmdldFgoKTtcbn07XG5cblJlY3RhbmdsZUQucHJvdG90eXBlLmdldE1heFggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5nZXRYKCkgKyB0aGlzLndpZHRoO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0Q2VudGVyWSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLnkgKyB0aGlzLmhlaWdodCAvIDI7XG59O1xuXG5SZWN0YW5nbGVELnByb3RvdHlwZS5nZXRNaW5ZID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMuZ2V0WSgpO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0TWF4WSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmdldFkoKSArIHRoaXMuaGVpZ2h0O1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0V2lkdGhIYWxmID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMud2lkdGggLyAyO1xufTtcblxuUmVjdGFuZ2xlRC5wcm90b3R5cGUuZ2V0SGVpZ2h0SGFsZiA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmhlaWdodCAvIDI7XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IFJlY3RhbmdsZUQ7XG4iLCJ2YXIgQ29TRUNvbnN0YW50cyA9IHJlcXVpcmUoJy4vQ29TRUNvbnN0YW50cycpO1xuXG5mdW5jdGlvbiBTYmduUERDb25zdGFudHMoKSB7XG59XG5cbi8vU2JnblBEQ29uc3RhbnRzIGluaGVyaXRzIHN0YXRpYyBwcm9wcyBpbiBDb1NFQ29uc3RhbnRzIFxuZm9yICh2YXIgcHJvcCBpbiBDb1NFQ29uc3RhbnRzKSB7XG4gIFNiZ25QRENvbnN0YW50c1twcm9wXSA9IENvU0VDb25zdGFudHNbcHJvcF07XG59XG5cbi8vIEJlbG93IGFyZSB0aGUgU0JHTiBnbHlwaCBzcGVjaWZpYyB0eXBlcy5cblNiZ25QRENvbnN0YW50cy5NQUNST01PTEVDVUxFID0gXCJtYWNyb21vbGVjdWxlXCI7XG5TYmduUERDb25zdGFudHMuVU5JVF9PRl9JTkZPUk1BVElPTiA9IFwidW5pdCBvZiBpbmZvcm1hdGlvblwiO1xuU2JnblBEQ29uc3RhbnRzLlNUQVRFX1ZBUklBQkxFID0gXCJzdGF0ZSB2YXJpYWJsZVwiO1xuU2JnblBEQ29uc3RhbnRzLlNPVVJDRV9BTkRfU0lOSyA9IFwic291cmNlIGFuZCBzaW5rXCI7XG5TYmduUERDb25zdGFudHMuQVNTT0NJQVRJT04gPSBcImFzc29jaWF0aW9uXCI7XG5TYmduUERDb25zdGFudHMuRElTU09DSUFUSU9OID0gXCJkaXNzb2NpYXRpb25cIjtcblNiZ25QRENvbnN0YW50cy5PTUlUVEVEX1BST0NFU1MgPSBcIm9taXR0ZWQgcHJvY2Vzc1wiO1xuU2JnblBEQ29uc3RhbnRzLlVOQ0VSVEFJTl9QUk9DRVNTID0gXCJ1bmNlcnRhaW4gcHJvY2Vzc1wiO1xuU2JnblBEQ29uc3RhbnRzLlNJTVBMRV9DSEVNSUNBTCA9IFwic2ltcGxlIGNoZW1pY2FsXCI7XG5TYmduUERDb25zdGFudHMuUFJPQ0VTUyA9IFwicHJvY2Vzc1wiO1xuU2JnblBEQ29uc3RhbnRzLkNPTVBMRVggPSBcImNvbXBsZXhcIjtcblNiZ25QRENvbnN0YW50cy5BTkQgPSBcImFuZFwiO1xuU2JnblBEQ29uc3RhbnRzLk9SID0gXCJvclwiO1xuU2JnblBEQ29uc3RhbnRzLk5PVCA9IFwibm90XCI7XG5TYmduUERDb25zdGFudHMuUEhFTk9UWVBFID0gXCJwaGVub3R5cGVcIjtcblNiZ25QRENvbnN0YW50cy5QRVJUVVJCSU5HX0FHRU5UID0gXCJwZXJ0dXJiaW5nIGFnZW50XCI7XG5TYmduUERDb25zdGFudHMuVEFHID0gXCJ0YWdcIjtcblNiZ25QRENvbnN0YW50cy5OVUNMRUlDX0FDSURfRkVBVFVSRSA9IFwibnVjbGVpYyBhY2lkIGZlYXR1cmVcIjtcblNiZ25QRENvbnN0YW50cy5VTlNQRUNJRklFRF9FTlRJVFkgPSBcInVuc3BlY2lmaWVkIGVudGl0eVwiO1xuU2JnblBEQ29uc3RhbnRzLklOUFVUX1BPUlQgPSBcImlucHV0X3BvcnRcIjtcblNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVCA9IFwib3V0cHV0X3BvcnRcIjtcblxuLyoqXG4gKiAgVGhpcyBjb21wb3VuZCB0eXBlIGlzIG9ubHkgdXNlZCB0byBlbmNsb3NlIGEgcHJvY2VzcyBub2RlIGFuZCBpdHMgdHdvIGFzc29jaWF0ZWQgcG9ydCBub2RlcyBcbiAqL1xuU2JnblBEQ29uc3RhbnRzLkRVTU1ZX0NPTVBPVU5EID0gXCJkdW1teSBjb21wb3VuZFwiO1xuXG4vLyBCZWxvdyBhcmUgdGhlIFNCR04gQXJjIHNwZWNpZmljIHR5cGVzLlxuU2JnblBEQ29uc3RhbnRzLlBST0RVQ1RJT04gPSBcInByb2R1Y3Rpb25cIjtcblNiZ25QRENvbnN0YW50cy5DT05TVU1QVElPTiA9IFwiY29uc3VtcHRpb25cIjtcblNiZ25QRENvbnN0YW50cy5JTkhJQklUSU9OID0gXCJpbmhpYml0aW9uXCI7XG5TYmduUERDb25zdGFudHMuQ0FUQUxZU0lTID0gXCJjYXRhbHlzaXNcIjtcblNiZ25QRENvbnN0YW50cy5NT0RVTEFUSU9OID0gXCJtb2R1bGF0aW9uXCI7XG5TYmduUERDb25zdGFudHMuU1RJTVVMQVRJT04gPSBcInN0aW11bGF0aW9uXCI7XG5TYmduUERDb25zdGFudHMuTkVDRVNTQVJZX1NUSU1VTEFUSU9OID0gXCJuZWNlc3Nhcnkgc3RpbXVsYXRpb25cIjtcblxuU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RIID0gMTA7XG5TYmduUERDb25zdGFudHMuUklHSURfRURHRSA9IFwicmlnaWQgZWRnZVwiO1xuXG5TYmduUERDb25zdGFudHMuUE9SVF9OT0RFX0RFRkFVTFRfV0lEVEggPSAzO1xuU2JnblBEQ29uc3RhbnRzLlBPUlRfTk9ERV9ERUZBVUxUX0hFSUdIVCA9IDM7XG5cblNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUiA9IDU7XG5TYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fVkVSVElDQUxfQlVGRkVSID0gNTtcblNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4gPSAyMDtcblNiZ25QRENvbnN0YW50cy5DT01QTEVYX01JTl9XSURUSCA9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4gKiAyO1x0XG5cblNiZ25QRENvbnN0YW50cy5QSEFTRTFfTUFYX0lURVJBVElPTl9DT1VOVCA9IDIwMDtcblNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX0RJU1RBTkNFID0gMTA7XG5TYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9QRVJJT0QgPSAzMDtcblNiZ25QRENvbnN0YW50cy5QSEFTRTJfSU5JVElBTF9DT09MSU5HRkFDVE9SID0gMC4zO1xuXG5TYmduUERDb25zdGFudHMuRUZGRUNUT1JfQU5HTEVfVE9MRVJBTkNFID0gNDU7XG5TYmduUERDb25zdGFudHMuQU5HTEVfVE9MRVJBTkNFID0gMTAwO1xuU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzkwX0RFR1JFRSA9IDYwO1xuU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzE4MF9ERUdSRUUgPSAwLjU7XG5TYmduUERDb25zdGFudHMuUk9UQVRJT05BTF9GT1JDRV9JVEVSQVRJT05fQ09VTlQgPSAyO1xuU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OQUxfRk9SQ0VfQ09OVkVSR0VOQ0UgPSAxLjA7XG5cbm1vZHVsZS5leHBvcnRzID0gU2JnblBEQ29uc3RhbnRzOyIsInZhciBDb1NFRWRnZSA9IHJlcXVpcmUoJy4vQ29TRUVkZ2UnKTtcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xuXG5mdW5jdGlvbiBTYmduUERFZGdlKHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSwgdHlwZSkgXG57XG4gICAgQ29TRUVkZ2UuY2FsbCh0aGlzLCBzb3VyY2UsIHRhcmdldCwgdkVkZ2UpO1xuICAgIFxuICAgIHRoaXMudHlwZSA9IHR5cGU7ICAgICAgICAgICAgICAgIC8vIFN0cmluZyAoZnJvbSBMR3JhcGhPYmplY3QpXG4gICAgdGhpcy5jb3JyZXNwb25kaW5nQW5nbGUgPSAwOyAgICAgLy8gaW50XG4gICAgdGhpcy5pc1Byb3Blcmx5T3JpZW50ZWQgPSBmYWxzZTsgLy8gYm9vbGVhblxufVxuXG5TYmduUERFZGdlLnByb3RvdHlwZSA9IE9iamVjdC5jcmVhdGUoQ29TRUVkZ2UucHJvdG90eXBlKTtcbmZvciAodmFyIHByb3AgaW4gQ29TRUVkZ2UpIHtcbiAgU2JnblBERWRnZVtwcm9wXSA9IENvU0VFZGdlW3Byb3BdO1xufVxuXG5TYmduUERFZGdlLnByb3RvdHlwZS5jb3B5ID0gZnVuY3Rpb24gKC8qU2JnblBERWRnZSovIGVkZ2UpIFxue1xuICAgIC8vIFRPRE86IERvIHdlIHJlYWxseSBoYXZlIGFjY2VzcyB0byB0aGVzZSB0d28gZnVuY3Rpb25zP1xuICAgIHRoaXMuc2V0U291cmNlKGVkZ2UuZ2V0U291cmNlKCkpO1xuICAgIHRoaXMuc2V0VGFyZ2V0KGVkZ2UuZ2V0VGFyZ2V0KCkpO1xuICAgIFxuICAgIHRoaXMubGFiZWwgPSBlZGdlLmxhYmVsO1xuICAgIHRoaXMudHlwZSA9IGVkZ2UudHlwZTtcbiAgICB0aGlzLmNvcnJlc3BvbmRpbmdBbmdsZSA9IGVkZ2UuY29ycmVzcG9uZGluZ0FuZ2xlO1xuICAgIHRoaXMuaXNQcm9wZXJseU9yaWVudGVkID0gZWRnZS5pc1Byb3Blcmx5T3JpZW50ZWQ7XG4gICAgdGhpcy5pZGVhbExlbmd0aCA9IGVkZ2UuaWRlYWxMZW5ndGg7XG4gICAgdGhpcy5pc0ludGVyR3JhcGggPSBlZGdlLmlzSW50ZXJHcmFwaDtcbiAgICB0aGlzLmJlbmRwb2ludHMgPSBlZGdlLmJlbmRwb2ludHM7XG4gICAgdGhpcy5pc092ZXJsYXBpbmdTb3VyY2VBbmRUYXJnZXQgPSBlZGdlLmlzT3ZlcmxhcGluZ1NvdXJjZUFuZFRhcmdldDtcbiAgICB0aGlzLmxjYSA9IGVkZ2UubGNhO1xuICAgIHRoaXMubGVuZ3RoID0gZWRnZS5sZW5ndGg7XG4gICAgdGhpcy5sZW5ndGhYID0gZWRnZS5sZW5ndGhYO1xuICAgIHRoaXMubGVuZ3RoWSA9IGVkZ2UubGVuZ3RoWTtcbiAgICB0aGlzLnNvdXJjZUluTGNhID0gZWRnZS5zb3VyY2VJbkxjYTtcbn07XG5cblNiZ25QREVkZ2UucHJvdG90eXBlLmlzRWZmZWN0b3IgPSBmdW5jdGlvbiAoKSBcbntcbiAgICBpZih0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuTU9EVUxBVElPTikgPT09IDAgfHwgXG4gICAgICAgdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLlNUSU1VTEFUSU9OKSA9PT0gMCB8fCBcbiAgICAgICB0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuQ0FUQUxZU0lTKSA9PT0gMCB8fCBcbiAgICAgICB0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuSU5ISUJJVElPTikgPT09IDAgfHwgXG4gICAgICAgdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLk5FQ0VTU0FSWV9TVElNVUxBVElPTikgPT09IDApXG4gICB7XG4gICAgICAgcmV0dXJuIHRydWU7XG4gICB9XG4gICBlbHNlXG4gICB7XG4gICAgICAgcmV0dXJuIGZhbHNlO1xuICAgfVxufTtcblxuU2JnblBERWRnZS5wcm90b3R5cGUuaXNSaWdpZEVkZ2UgPSBmdW5jdGlvbiAoKSBcbntcbiAgICBpZih0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuUklHSURfRURHRSkgPT09IDAgKVxuICAgIHtcbiAgICAgICAgcmV0dXJuIHRydWU7XG4gICAgfVxuICAgIGVsc2VcbiAgICB7XG4gICAgICAgIHJldHVybiBmYWxzZTtcbiAgICB9XG59O1xuXG5tb2R1bGUuZXhwb3J0cyA9IFNiZ25QREVkZ2U7XG4iLCJ2YXIgSW50ZWdlciA9IHJlcXVpcmUoJy4vSW50ZWdlcicpO1xyXG52YXIgSUdlb21ldHJ5ID0gcmVxdWlyZSgnLi9JR2VvbWV0cnknKTtcclxudmFyIFBvaW50RCA9IHJlcXVpcmUoJy4vUG9pbnREJyk7XHJcbnZhciBSZWN0YW5nbGVEID0gcmVxdWlyZSgnLi9SZWN0YW5nbGVEJyk7XHJcblxyXG52YXIgSGFzaE1hcCA9IHJlcXVpcmUoJy4vSGFzaE1hcCcpO1xyXG52YXIgSGFzaFNldCA9IHJlcXVpcmUoJy4vSGFzaFNldCcpO1xyXG5cclxudmFyIENvU0VMYXlvdXQgPSByZXF1aXJlKCcuL0NvU0VMYXlvdXQnKTtcclxudmFyIFNiZ25QRE5vZGUgPSByZXF1aXJlKCcuL1NiZ25QRE5vZGUnKTtcclxudmFyIFNiZ25QREVkZ2UgPSByZXF1aXJlKCcuL1NiZ25QREVkZ2UnKTtcclxudmFyIFNiZ25Qcm9jZXNzTm9kZSA9IHJlcXVpcmUoJy4vU2JnblByb2Nlc3NOb2RlJyk7XHJcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xyXG5cclxudmFyIE1lbWJlclBhY2sgPSByZXF1aXJlKCcuL01lbWJlclBhY2snKTtcclxudmFyIFJlY3RQcm9jID0gcmVxdWlyZSgnLi9SZWN0UHJvYycpO1xyXG52YXIgQ29tcGFjdGlvbiA9IHJlcXVpcmUoJy4vQ29tcGFjdGlvbicpO1xyXG5cclxuU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bSA9IFxyXG57XHJcbiAgICBUSUxJTkcgOiAwLCBcclxuICAgIFBPTFlPTUlOT19QQUNLSU5HIDogMVxyXG59O1xyXG5cclxuU2JnblBETGF5b3V0Lk9yaWVudGF0aW9uRW51bSA9IFxyXG57XHJcbiAgICBMRUZUX1RPX1JJR0hUIDogMCwgXHJcbiAgICBSSUdIVF9UT19MRUZUIDogMSxcclxuICAgIFRPUF9UT19CT1RUT00gOiAyLCBcclxuICAgIEJPVFRPTV9UT19UT1AgOiAzXHJcbn07XHJcblxyXG5mdW5jdGlvbiBTYmduUERMYXlvdXQoKSBcclxue1xyXG4gICAgQ29TRUxheW91dC5jYWxsKHRoaXMpO1xyXG5cclxuICAgIHRoaXMucm90YXRpb25SYW5kb21pemF0aW9uTWV0aG9kID0gMTtcclxuXHJcbiAgICB0aGlzLmVuaGFuY2VkUmF0aW8gPSAwO1xyXG4gICAgdGhpcy50b3RhbEVmZkNvdW50ID0gMDtcclxuICAgIHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9IFNiZ25QRExheW91dC5EZWZhdWx0Q29tcGFjdGlvbkFsZ29yaXRobUVudW0uVElMSU5HO1xyXG5cclxuICAgIHRoaXMuY2hpbGRHcmFwaE1hcCA9IG5ldyBIYXNoTWFwKCk7ICAgICAgICAgIC8qTWFwPFNiZ25QRE5vZGUsIExHcmFwaD4qL1xyXG4gICAgdGhpcy5jb21wbGV4T3JkZXIgPSBbXTsgICAgICAgICAgICAgICAgICAgICAgLypMaXN0PFNiZ25QRE5vZGU+Ki9cclxuICAgIHRoaXMuZHVtbXlDb21wbGV4TGlzdCA9IFtdOyAgICAgICAgICAgICAgICAgIC8qTGlzdDxTYmduUEROb2RlPiovXHJcbiAgICB0aGlzLmVtcHRpZWREdW1teUNvbXBsZXhNYXAgPSBuZXcgSGFzaE1hcCgpOyAvKk1hcDxTYmduUEROb2RlLCBMR3JhcGg+Ki9cclxuICAgIHRoaXMucHJvY2Vzc05vZGVMaXN0ID0gW107ICAgICAgICAgICAgICAgICAgIC8qTGlzdDxTYmduUHJvY2Vzc05vZGU+Ki9cclxuICAgIFxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5USUxJTkcpXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5tZW1iZXJQYWNrTWFwID0gbmV3IEhhc2hNYXAoKTsgICAgICAvKk1hcDxTYmduUEROb2RlLCBNZW1iZXJQYWNrPiovXHJcbiAgICB9XHJcbn07XHJcblxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShDb1NFTGF5b3V0LnByb3RvdHlwZSk7XHJcblxyXG5mb3IgKHZhciBwcm9wIGluIENvU0VMYXlvdXQpIHtcclxuICBTYmduUERMYXlvdXRbcHJvcF0gPSBDb1NFTGF5b3V0W3Byb3BdO1xyXG59XHJcblxyXG4vKipcclxuICogQE92ZXJyaWRlIFRoaXMgbWV0aG9kIHBlcmZvcm1zIHRoZSBhY3R1YWwgbGF5b3V0IG9uIHRoZSBsLWxldmVsIGNvbXBvdW5kXHJcbiAqICAgICAgICAgICBncmFwaC4gQW4gdXBkYXRlKCkgbmVlZHMgdG8gYmUgY2FsbGVkIGZvciBjaGFuZ2VzIHRvIGJlXHJcbiAqICAgICAgICAgICBwcm9wYWdhdGVkIHRvIHRoZSB2LWxldmVsIGNvbXBvdW5kIGdyYXBoLlxyXG4gKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5ydW5TcHJpbmdFbWJlZGRlciA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIGNvbnNvbGUubG9nKFwiU0JHTi1QRCBMYXlvdXQgaXMgcnVubmluZy4uLlwiKTtcclxuICAgIHRoaXMucGhhc2VOdW1iZXIgPSAxO1xyXG4gICAgdGhpcy5kb1BoYXNlMSgpO1xyXG5cclxuICAgIHRoaXMucGhhc2VOdW1iZXIgPSAyO1xyXG4gICAgdGhpcy5kb1BoYXNlMigpO1xyXG5cclxuICAgIC8vIHVzZWQgdG8gY2FsY3VsYXRlIC0gdG8gbWFrZSBzdXJlXHJcbiAgICB0aGlzLnJlY2FsY1Byb3Blcmx5T3JpZW50ZWRFZGdlcyh0cnVlKTtcclxuXHJcbiAgICBjb25zb2xlLmxvZyhcInN1Y2Nlc3MgcmF0aW86IFwiICsgdGhpcy5zdWNjZXNzUmF0aW8pO1xyXG5cclxuICAgIHRoaXMuZmluYWxFbmhhbmNlbWVudCgpO1xyXG5cclxuICAgIGNvbnNvbGUubG9nKFwiZW5oYW5jZWQgcmF0aW86IFwiICsgdGhpcy5lbmhhbmNlZFJhdGlvKTtcclxuXHJcbiAgICB0aGlzLnJlbW92ZUR1bW15Q29tcG91bmRzKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBBdCB0aGlzIHBoYXNlLCBDb1NFIGlzIGFwcGxpZWQgZm9yIGEgbnVtYmVyIG9mIGl0ZXJhdGlvbnMuXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuZG9QaGFzZTEgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB0aGlzLm1heEl0ZXJhdGlvbnMgPSBTYmduUERDb25zdGFudHMuUEhBU0UxX01BWF9JVEVSQVRJT05fQ09VTlQ7XHJcbiAgICB0aGlzLnRvdGFsSXRlcmF0aW9ucyA9IDA7XHJcblxyXG4gICAgZG9cclxuICAgIHtcclxuICAgICAgICB0aGlzLnRvdGFsSXRlcmF0aW9ucysrO1xyXG4gICAgICAgIGlmICgodGhpcy50b3RhbEl0ZXJhdGlvbnMgJSBTYmduUERDb25zdGFudHMuQ09OVkVSR0VOQ0VfQ0hFQ0tfUEVSSU9EKSA9PT0gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0aGlzLmlzQ29udmVyZ2VkKCkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICB0aGlzLmNvb2xpbmdGYWN0b3IgPSBcclxuICAgICAgICAgICAgICAgIHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3IgKiBcclxuICAgICAgICAgICAgICAgICgodGhpcy5tYXhJdGVyYXRpb25zIC0gdGhpcy50b3RhbEl0ZXJhdGlvbnMpIC8gdGhpcy5tYXhJdGVyYXRpb25zKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHRoaXMudG90YWxEaXNwbGFjZW1lbnQgPSAwO1xyXG5cclxuICAgICAgICB0aGlzLmdyYXBoTWFuYWdlci51cGRhdGVCb3VuZHMoKTtcclxuICAgICAgICB0aGlzLmNhbGNTcHJpbmdGb3JjZXMoKTtcclxuICAgICAgICB0aGlzLmNhbGNSZXB1bHNpb25Gb3JjZXMoKTtcclxuICAgICAgICB0aGlzLmNhbGNHcmF2aXRhdGlvbmFsRm9yY2VzKCk7XHJcbiAgICAgICAgdGhpcy5tb3ZlTm9kZXMoKTtcclxuXHJcbiAgICAgICAgdGhpcy5hbmltYXRlKCk7XHJcbiAgICB9XHJcbiAgICB3aGlsZSAodGhpcy50b3RhbEl0ZXJhdGlvbnMgPCB0aGlzLm1heEl0ZXJhdGlvbnMpO1xyXG5cclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnVwZGF0ZUJvdW5kcygpO1xyXG4gICAgdGhpcy5waGFzZTFJdGVyYXRpb25Db3VudCA9IHRoaXMudG90YWxJdGVyYXRpb25zO1xyXG59O1xyXG5cclxuLyoqXHJcbiogQXQgdGhpcyBwaGFzZSwgbG9jYXRpb24gb2Ygc2luZ2xlIG5vZGVzIGFyZSBhcHByb3hpbWF0ZWQgb2NjYXNpb25hbGx5LlxyXG4qIFJvdGF0aW9uYWwgZm9yY2VzIGFyZSBhcHBsaWVkLiBDb29saW5nIGZhY3RvciBzdGFydHMgZnJvbSBhIHNtYWxsIHZhbHVlXHJcbiogdG8gcHJldmVudCBodWdlIGNoYW5nZXMuXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuZG9QaGFzZTIgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICAvLyBkeW5hbWljIG1heCBpdGVyYXRpb25cclxuICAgIHRoaXMubWF4SXRlcmF0aW9ucyA9IFxyXG4gICAgICAgICAgICBNYXRoLmxvZyh0aGlzLmdldEFsbEVkZ2VzKCkubGVuZ3RoICsgdGhpcy5nZXRBbGxOb2RlcygpLmxlbmd0aCkgKiA0MDA7XHJcblxyXG4gICAgLy8gY29vbGluZyBmYWMgaXMgc21hbGxcclxuICAgIHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3IgPSBTYmduUERDb25zdGFudHMuUEhBU0UyX0lOSVRJQUxfQ09PTElOR0ZBQ1RPUjtcclxuICAgIHRoaXMuY29vbGluZ0ZhY3RvciA9IHRoaXMuaW5pdGlhbENvb2xpbmdGYWN0b3I7XHJcblxyXG4gICAgdGhpcy50b3RhbEl0ZXJhdGlvbnMgPSAwO1xyXG5cclxuICAgIGRvXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy50b3RhbEl0ZXJhdGlvbnMrKztcclxuXHJcbiAgICAgICAgaWYgKCh0aGlzLnRvdGFsSXRlcmF0aW9ucyAlIFNiZ25QRENvbnN0YW50cy5DT05WRVJHRU5DRV9DSEVDS19QRVJJT0QpID09PSAwKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMuc3VjY2Vzc1JhdGlvID0gdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50IC8gdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZDtcclxuXHJcbiAgICAgICAgICAgICAgICBpZiAodGhpcy5pc0NvbnZlcmdlZCgpICYmIFxyXG4gICAgICAgICAgICAgICAgICAgIHRoaXMuc3VjY2Vzc1JhdGlvID49IFNiZ25QRENvbnN0YW50cy5ST1RBVElPTkFMX0ZPUkNFX0NPTlZFUkdFTkNFKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgICAgIHRoaXMuY29vbGluZ0ZhY3RvciA9IFxyXG4gICAgICAgICAgICAgICAgICAgICAgICB0aGlzLmluaXRpYWxDb29saW5nRmFjdG9yICogXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICgodGhpcy5tYXhJdGVyYXRpb25zIC0gdGhpcy50b3RhbEl0ZXJhdGlvbnMpIC8gdGhpcy5tYXhJdGVyYXRpb25zKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHRoaXMudG90YWxEaXNwbGFjZW1lbnQgPSAwO1xyXG5cclxuICAgICAgICB0aGlzLmdyYXBoTWFuYWdlci51cGRhdGVCb3VuZHMoKTtcclxuXHJcbiAgICAgICAgdGhpcy5jYWxjU3ByaW5nRm9yY2VzKCk7XHJcbiAgICAgICAgdGhpcy5jYWxjUmVwdWxzaW9uRm9yY2VzKCk7XHJcbiAgICAgICAgdGhpcy5jYWxjR3Jhdml0YXRpb25hbEZvcmNlcygpO1xyXG4gICAgICAgIHRoaXMubW92ZU5vZGVzKCk7XHJcbiAgICAgICAgdGhpcy5hbmltYXRlKCk7XHJcbiAgICB9XHJcbiAgICB3aGlsZSAodGhpcy50b3RhbEl0ZXJhdGlvbnMgPCB0aGlzLm1heEl0ZXJhdGlvbnNcclxuICAgICAgICAgICAgICAgICAgICAmJiB0aGlzLnRvdGFsSXRlcmF0aW9ucyA8IDEwMDAwKTtcclxuXHJcbiAgICB0aGlzLnBoYXNlMkl0ZXJhdGlvbkNvdW50ID0gdGhpcy50b3RhbEl0ZXJhdGlvbnM7XHJcbiAgICB0aGlzLmdyYXBoTWFuYWdlci51cGRhdGVCb3VuZHMoKTtcclxufTtcclxuXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUubW92ZU5vZGVzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50ID0gMDtcclxuICAgIHRoaXMudG90YWxFZGdlQ291bnRUb0JlT3JpZW50ZWQgPSAwO1xyXG5cclxuICAgIC8vIG9ubHkgY2hhbmdlIHNpbmdsZSBub2RlIHBvc2l0aW9ucyBvbiBlYXJseSBzdGFnZXNcclxuICAgIGlmICh0aGlzLmhhc0FwcHJveGltYXRpb25QZXJpb2RSZWFjaGVkKCkgJiYgdGhpcy5jb29saW5nRmFjdG9yID4gMC4wMilcclxuICAgIHtcclxuICAgICAgICB2YXIgbnVtT2ZQcm9jZXNzTm9kZXMgPSB0aGlzLnByb2Nlc3NOb2RlTGlzdC5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgaW5kZXg7IGluZGV4PG51bU9mUHJvY2Vzc05vZGVzOyBpbmRleCsrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGhpcy5wcm9jZXNzTm9kZUxpc3RbaW5kZXhdLmFwcGx5QXBwcm94aW1hdGlvbnMoKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBcclxuICAgIHZhciBudW1PZlByb2Nlc3NOb2RlcyA9IHRoaXMucHJvY2Vzc05vZGVMaXN0Lmxlbmd0aDtcclxuICAgIGZvciAodmFyIGluZGV4OyBpbmRleDxudW1PZlByb2Nlc3NOb2RlczsgaW5kZXgrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgc2JnblByb2Nlc3NOb2RlID0gdGhpcy5wcm9jZXNzTm9kZUxpc3RbaW5kZXhdO1xyXG4gICAgICAgIFxyXG4gICAgICAgIC8vIGNhbGN1bGF0ZSByb3RhdGlvbmFsIGZvcmNlcyBmb3IgcGhhc2UgMiBvbmx5XHJcbiAgICAgICAgaWYgKHRoaXMucGhhc2VOdW1iZXIgPT09IDIpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUuY2FsY1JvdGF0aW9uYWxGb3JjZXMoKTtcclxuXHJcbiAgICAgICAgICAgIHRoaXMucHJvcGVybHlPcmllbnRlZEVkZ2VDb3VudCArPSBzYmduUHJvY2Vzc05vZGUucHJvcGVyRWRnZUNvdW50O1xyXG4gICAgICAgICAgICB0aGlzLnRvdGFsRWRnZUNvdW50VG9CZU9yaWVudGVkICs9IFxyXG4gICAgICAgICAgICAgICAgICAgIChzYmduUHJvY2Vzc05vZGUuY29uc3VtcHRpb25FZGdlcy5zaXplKCkgKyBcclxuICAgICAgICAgICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLnByb2R1Y3RFZGdlcy5zaXplKCkgKyBcclxuICAgICAgICAgICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLmVmZmVjdG9yRWRnZXMuc2l6ZSgpKTtcclxuICAgICAgICAgICAgdGhpcy5zdWNjZXNzUmF0aW8gPSBcclxuICAgICAgICAgICAgICAgICAgICB0aGlzLnByb3Blcmx5T3JpZW50ZWRFZGdlQ291bnQgLyB0aGlzLnRvdGFsRWRnZUNvdW50VG9CZU9yaWVudGVkO1xyXG4gICAgICAgIH1cclxuICAgICAgICBcclxuICAgICAgICBzYmduUHJvY2Vzc05vZGUudHJhbnNmZXJGb3JjZXMoKTtcclxuXHJcbiAgICAgICAgc2JnblByb2Nlc3NOb2RlLnJlc2V0Rm9yY2VzKCk7XHJcbiAgICAgICAgc2JnblByb2Nlc3NOb2RlLmlucHV0UG9ydC5yZXNldEZvcmNlcygpO1xyXG4gICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5vdXRwdXRQb3J0LnJlc2V0Rm9yY2VzKCk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZWFjaCB0aW1lLCByb3RhdGUgb25lIHByb2Nlc3MgdGhhdCB3YW50cyB0byByb3RhdGVcclxuICAgIGlmICgoKHRoaXMudG90YWxJdGVyYXRpb25zICUgU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OQUxfRk9SQ0VfSVRFUkFUSU9OX0NPVU5UKSA9PT0gMCkgJiYgXHJcbiAgICAgICAgdGhpcy5waGFzZU51bWJlciA9PT0gMilcclxuICAgIHtcclxuICAgICAgICB0aGlzLnJvdGF0ZUFQcm9jZXNzKCk7XHJcbiAgICB9XHJcbiAgICAgICAgICAgIFxyXG4gICAgQ29TRUxheW91dC5wcm90b3R5cGUubW92ZU5vZGVzLmNhbGwodGhpcyk7XHJcbn07XHJcblxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmhhc0FwcHJveGltYXRpb25QZXJpb2RSZWFjaGVkID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgaWYoKHRoaXMudG90YWxJdGVyYXRpb25zICUgMTAwKSA9PT0gKFNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX1BFUklPRCkpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5yb3RhdGVBUHJvY2VzcyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBwcm9jZXNzTm9kZXNUb0JlUm90YXRlZCA9IFtdIC8qTGlzdDxTYmduUHJvY2Vzc05vZGU+Ki87XHJcbiAgICBcclxuICAgIHZhciBudW1PZlByb2Nlc3NOb2RlcyA9IHRoaXMucHJvY2Vzc05vZGVMaXN0Lmxlbmd0aDtcclxuICAgIGZvciAodmFyIGluZGV4OyBpbmRleDxudW1PZlByb2Nlc3NOb2RlczsgaW5kZXgrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgc2JnblByb2Nlc3NOb2RlID0gdGhpcy5wcm9jZXNzTm9kZUxpc3RbaW5kZXhdO1xyXG4gICAgICAgIFxyXG4gICAgICAgIGlmIChzYmduUHJvY2Vzc05vZGUuaXNSb3RhdGlvbk5lY2Vzc2FyeSgpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQucHVzaChzYmduUHJvY2Vzc05vZGUpO1xyXG4gICAgICAgIH0gICAgICAgXHJcbiAgICB9XHJcblxyXG4gICAgLy8gcmFuZG9tIHNlbGVjdGlvblxyXG4gICAgaWYgKHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkLmxlbmd0aCA+IDApXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHJhbmRvbUluZGV4ID0gMDtcclxuXHJcbiAgICAgICAgaWYgKHRoaXMucm90YXRpb25SYW5kb21pemF0aW9uTWV0aG9kID09PSAwKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmFuZG9tSW5kZXggPSB0aGlzLnJvdWxldHRlV2hlZWxTZWxlY3Rpb24ocHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQpO1xyXG5cclxuICAgICAgICAgICAgaWYgKHJhbmRvbUluZGV4ID09PSAtMSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgY29uc29sZS5sb2coXCJFUlJPUjogbm8gbm9kZXMgaGF2ZSBiZWVuIHNlbGVjdGVkIGZvciByb3RhdGlvblwiKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByYW5kb21JbmRleCA9IChNYXRoLnJhbmRvbSgpICogcHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQubGVuZ3RoKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHZhciBzYmduUHJvY2Vzc05vZGUgPSBwcm9jZXNzTm9kZXNUb0JlUm90YXRlZFtyYW5kb21JbmRleF07XHJcbiAgICAgICAgc2JnblByb2Nlc3NOb2RlLmFwcGx5Um90YXRpb24oKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyAvLyByZXNldCBuZXQgcm90YXRpb25hbCBmb3JjZXMgb24gYWxsIHByb2Nlc3NlcyBmb3IgbmV4dCByb3VuZFxyXG4gICAgLy8gbm90IHVzZWQgYmVjYXVzZSBldmVuIGlmIHRoZSBhbW91bnQgaXMgc21hbGwsIHN1bW1pbmcgdXAgdGhlIG5ldFxyXG4gICAgLy8gZm9yY2UgZnJvbSBwcmV2IGl0ZXJhdGlvbnMgeWllbGQgYmV0dGVyIHJlc3VsdHNcclxuICAgIC8vIGZvciAoT2JqZWN0IG8gOiB0aGlzLmdldEFsbE5vZGVzKCkpXHJcbiAgICAvLyB7XHJcbiAgICAvLyBpZiAobyBpbnN0YW5jZW9mIFNiZ25Qcm9jZXNzTm9kZSlcclxuICAgIC8vICgoU2JnblByb2Nlc3NOb2RlKSBvKS5uZXRSb3RhdGlvbmFsRm9yY2UgPSAwO1xyXG4gICAgLy8gfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgaXRlcmF0ZXMgb3ZlciB0aGUgcHJvY2VzcyBub2RlcyBhbmQgY2hlY2tzIGlmIHRoZXJlIGV4aXN0c1xyXG4qIGFub3RoZXIgb3JpZW50YXRpb24gd2hpY2ggbWF4aW1pemVzIHRoZSB0b3RhbCBudW1iZXIgb2YgcHJvcGVybHkgZWRnZXMuXHJcbiogSWYgdGhlcmUgaXMsIHRoZSBvcmllbnRhdGlvbiBpcyBjaGFuZ2VkLlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmZpbmFsRW5oYW5jZW1lbnQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgb3JpZW50YXRpb25MaXN0ID0gW10gLypMaXN0PE9yaWVudGF0aW9uPiovO1xyXG4gICAgdmFyIGJlc3RTdGVwUmVzdWx0ID0gMC4wO1xyXG4gICAgdmFyIGJlc3RPcmllbnRhdGlvbiA9IG51bGwgLypPcmllbnRhdGlvbiovO1xyXG4gICAgdmFyIHN0ZXBBcHByb3ByaWF0ZUVkZ2VDbnQgPSAwLjA7XHJcbiAgICB2YXIgdG90YWxQcm9wZXJFZGdlcyA9IDAuMDtcclxuICAgIHZhciBhbmdsZSA9IDAuMDtcclxuXHJcbiAgICBvcmllbnRhdGlvbkxpc3QucHVzaChTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQpO1xyXG4gICAgb3JpZW50YXRpb25MaXN0LnB1c2goU2JnblBETGF5b3V0Lk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUKTtcclxuICAgIG9yaWVudGF0aW9uTGlzdC5wdXNoKFNiZ25QRExheW91dC5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTSk7XHJcbiAgICBvcmllbnRhdGlvbkxpc3QucHVzaChTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLkJPVFRPTV9UT19UT1ApO1xyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZQcm9jZXNzTm9kZXMgPSB0aGlzLnByb2Nlc3NOb2RlTGlzdC5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpOyBpPG51bU9mUHJvY2Vzc05vZGVzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHNiZ25Qcm9jZXNzTm9kZSA9IHRoaXMucHJvY2Vzc05vZGVMaXN0W2ldO1xyXG4gICAgXHJcbiAgICAgICAgYmVzdFN0ZXBSZXN1bHQgPSBzYmduUHJvY2Vzc05vZGUucHJvcGVyRWRnZUNvdW50O1xyXG4gICAgICAgIGJlc3RPcmllbnRhdGlvbiA9IG51bGw7XHJcbiAgICAgICAgdmFyIHJlbWVtYmVyUHJvcExpc3QgPSBbXS8qTGlzdDxCb29sZWFuPiovO1xyXG4gICAgICAgIHZhciBiZXN0UHJvcExpc3QgPSBbXS8qTGlzdDxCb29sZWFuPiovO1xyXG4gICAgICAgIFxyXG4gICAgICAgIHZhciBudW1PZk9yaWVudGF0aW9ucyA9IG9yaWVudGF0aW9uTGlzdC5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgajsgajxudW1PZk9yaWVudGF0aW9uczsgaisrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdmFyIG9yaWVudCA9IG9yaWVudGF0aW9uTGlzdFtqXTtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIHN0ZXBBcHByb3ByaWF0ZUVkZ2VDbnQgPSAwO1xyXG5cclxuICAgICAgICAgICAgdmFyIGlucHV0UG9ydFRhcmdldCA9IHNiZ25Qcm9jZXNzTm9kZS5maW5kUG9ydFRhcmdldFBvaW50KHRydWUsIG9yaWVudCk7XHJcbiAgICAgICAgICAgIHZhciBvdXRwdXRQb3J0VGFyZ2V0ID0gc2JnblByb2Nlc3NOb2RlLmZpbmRQb3J0VGFyZ2V0UG9pbnQoZmFsc2UsIG9yaWVudCk7XHJcblxyXG4gICAgICAgICAgICByZW1lbWJlclByb3BMaXN0ID0gW107XHJcbiAgICAgICAgICAgIFxyXG4gICAgICAgICAgICB2YXIgbnVtT2ZDb25zdW1wdGlvbkVkZ2VzID0gc2JnblByb2Nlc3NOb2RlLmNvbnN1bXB0aW9uRWRnZXMubGVuZ3RoO1xyXG4gICAgICAgICAgICBmb3IgKHZhciBrOyBrPG51bU9mQ29uc3VtcHRpb25FZGdlczsgaysrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB2YXIgZWRnZSA9IHNiZ25Qcm9jZXNzTm9kZS5jb25zdW1wdGlvbkVkZ2VzW2tdO1xyXG4gICAgICAgICAgICAgICAgdmFyIG5vZGUgPSBlZGdlLmdldFNvdXJjZSgpO1xyXG4gICAgICAgICAgICAgICAgYW5nbGUgPSBJR2VvbWV0cnkuY2FsY3VsYXRlQW5nbGUoaW5wdXRQb3J0VGFyZ2V0LFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5pbnB1dFBvcnQuZ2V0Q2VudGVyKCksIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIG5vZGUuZ2V0Q2VudGVyKCkpO1xyXG4gICAgICAgICAgICAgICAgaWYgKGFuZ2xlIDw9IFNiZ25QRENvbnN0YW50cy5BTkdMRV9UT0xFUkFOQ0UpXHJcbiAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgc3RlcEFwcHJvcHJpYXRlRWRnZUNudCsrO1xyXG4gICAgICAgICAgICAgICAgICAgIHJlbWVtYmVyUHJvcExpc3QucHVzaCh0cnVlKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICByZW1lbWJlclByb3BMaXN0LnB1c2goZmFsc2UpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIFxyXG4gICAgICAgICAgICB2YXIgbnVtT2ZQcm9kdWN0RWRnZXMgPSBzYmduUHJvY2Vzc05vZGUucHJvZHVjdEVkZ2VzLmxlbmd0aDtcclxuICAgICAgICAgICAgZm9yICh2YXIgazsgazxudW1PZlByb2R1Y3RFZGdlczsgaysrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB2YXIgZWRnZSA9IHNiZ25Qcm9jZXNzTm9kZS5wcm9kdWN0RWRnZXNba107XHJcbiAgICAgICAgICAgICAgICB2YXIgbm9kZSA9IGVkZ2UuZ2V0VGFyZ2V0KCk7XHJcbiAgICAgICAgICAgICAgICBhbmdsZSA9IElHZW9tZXRyeS5jYWxjdWxhdGVBbmdsZShvdXRwdXRQb3J0VGFyZ2V0LFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5vdXRwdXRQb3J0LmdldENlbnRlcigpLCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlLmdldENlbnRlcigpKTtcclxuXHJcbiAgICAgICAgICAgICAgICBpZiAoYW5nbGUgPD0gU2JnblBEQ29uc3RhbnRzLkFOR0xFX1RPTEVSQU5DRSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBzdGVwQXBwcm9wcmlhdGVFZGdlQ250Kys7XHJcbiAgICAgICAgICAgICAgICAgICAgcmVtZW1iZXJQcm9wTGlzdC5wdXNoKHRydWUpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIHJlbWVtYmVyUHJvcExpc3QucHVzaChmYWxzZSk7XHJcbiAgICAgICAgICAgICAgICB9ICAgICAgICAgXHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIHZhciBudW1PZkVmZmVjdG9yRWRnZXMgPSBzYmduUHJvY2Vzc05vZGUuZWZmZWN0b3JFZGdlcy5sZW5ndGg7XHJcbiAgICAgICAgICAgIGZvciAodmFyIGs7IGs8bnVtT2ZFZmZlY3RvckVkZ2VzOyBrKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHZhciBlZGdlID0gc2JnblByb2Nlc3NOb2RlLmVmZmVjdG9yRWRnZXNba107XHJcbiAgICAgICAgICAgICAgICB2YXIgbm9kZSA9IGVkZ2UuZ2V0U291cmNlKCk7XHJcbiAgICAgICAgICAgICAgICBhbmdsZSA9IGNhbGNFZmZlY3RvckFuZ2xlKG9yaWVudCwgc2JnblByb2Nlc3NOb2RlLmdldENlbnRlcigpLCBub2RlKTtcclxuXHJcbiAgICAgICAgICAgICAgICBpZiAoYW5nbGUgPD0gU2JnblBEQ29uc3RhbnRzLkVGRkVDVE9SX0FOR0xFX1RPTEVSQU5DRSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBzdGVwQXBwcm9wcmlhdGVFZGdlQ250Kys7XHJcbiAgICAgICAgICAgICAgICAgICAgcmVtZW1iZXJQcm9wTGlzdC5wdXNoKHRydWUpO1xyXG5cclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIGVsc2VcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICByZW1lbWJlclByb3BMaXN0LnB1c2goZmFsc2UpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICBpZiAoc3RlcEFwcHJvcHJpYXRlRWRnZUNudCA+IGJlc3RTdGVwUmVzdWx0KVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBiZXN0U3RlcFJlc3VsdCA9IHN0ZXBBcHByb3ByaWF0ZUVkZ2VDbnQ7XHJcbiAgICAgICAgICAgICAgICBiZXN0T3JpZW50YXRpb24gPSBvcmllbnQ7XHJcbiAgICAgICAgICAgICAgICBiZXN0UHJvcExpc3QgPSByZW1lbWJlclByb3BMaXN0O1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHRvdGFsUHJvcGVyRWRnZXMgKz0gYmVzdFN0ZXBSZXN1bHQ7XHJcblxyXG4gICAgICAgIC8vIGl0IG1lYW5zIGEgYmV0dGVyIHBvc2l0aW9uIGhhcyBiZWVuIGZvdW5kXHJcbiAgICAgICAgaWYgKGJlc3RTdGVwUmVzdWx0ID4gc2JnblByb2Nlc3NOb2RlLnByb3BlckVkZ2VDb3VudClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5zZXRPcmllbnRhdGlvbihiZXN0T3JpZW50YXRpb24pO1xyXG4gICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUucHJvcGVyRWRnZUNvdW50ID0gYmVzdFN0ZXBSZXN1bHQ7XHJcblxyXG4gICAgICAgICAgICAvLyBtYXJrIGVkZ2VzIHdpdGggYmVzdCBrbm93biBjb25maWd1cmF0aW9uIHZhbHVlc1xyXG4gICAgICAgICAgICBmb3IgKHZhciBtID0gMDsgbSA8IHNiZ25Qcm9jZXNzTm9kZS5jb25zdW1wdGlvbkVkZ2VzLmxlbmd0aDsgbSsrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUuY29uc3VtcHRpb25FZGdlc1tpXS5pc1Byb3Blcmx5T3JpZW50ZWQgPSBiZXN0UHJvcExpc3RbaV07XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZm9yICh2YXIgbiA9IDA7IG4gPCBzYmduUHJvY2Vzc05vZGUucHJvZHVjdEVkZ2VzLmxlbmd0aDsgbisrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUucHJvZHVjdEVkZ2VzW2ldLmlzUHJvcGVybHlPcmllbnRlZCA9IGJlc3RQcm9wTGlzdFtpICsgc2JnblByb2Nlc3NOb2RlLmNvbnN1bXB0aW9uRWRnZXMubGVuZ3RoXTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBmb3IgKHZhciBvID0gMDsgbyA8IHNiZ25Qcm9jZXNzTm9kZS5lZmZlY3RvckVkZ2VzLnNpemUoKTsgbysrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLmVmZmVjdG9yRWRnZXNbaV0uaXNQcm9wZXJseU9yaWVudGVkID0gXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBiZXN0UHJvcExpc3RbaSArIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIChzYmduUHJvY2Vzc05vZGUuY29uc3VtcHRpb25FZGdlcy5sZW5ndGggKyBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgc2JnblByb2Nlc3NOb2RlLnByb2R1Y3RFZGdlcy5sZW5ndGgpXTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICB0aGlzLnByb3Blcmx5T3JpZW50ZWRFZGdlQ291bnQgPSB0b3RhbFByb3BlckVkZ2VzO1xyXG4gICAgdGhpcy5lbmhhbmNlZFJhdGlvID0gdG90YWxQcm9wZXJFZGdlcyAvIHRoaXMudG90YWxFZGdlQ291bnRUb0JlT3JpZW50ZWQ7XHJcblxyXG4gICAgdmFyIG51bU9mUHJvY2Vzc05vZGVzID0gdGhpcy5wcm9jZXNzTm9kZUxpc3QubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaTsgaTxudW1PZlByb2Nlc3NOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMudG90YWxFZmZDb3VudCArPSB0aGlzLnByb2Nlc3NOb2RlTGlzdFtpXS5lZmZlY3RvckVkZ2VzLmxlbmd0aDtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIElmIGEgcHJvY2VzcyBub2RlIGhhcyBoaWdoZXIgbmV0Um90YXRpb25hbEZvcmNlLCBpdCBoYXMgbW9yZSBjaGFuY2UgdG8gYmVcclxuKiByb3RhdGVkXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUucm91bGV0dGVXaGVlbFNlbGVjdGlvbiA9IGZ1bmN0aW9uIChcclxuICAgICAgICAgICAgICAgLypBcnJheUxpc3Q8U2JnblByb2Nlc3NOb2RlPiovIHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkKVxyXG57XHJcbiAgICB2YXIgcmFuZG9tTnVtYmVyID0gTWF0aC5yYW5kb20oKTtcclxuICAgIHZhciBmaXRuZXNzVmFsdWVzID0gW10gLypwcm9jZXNzTm9kZXNUb0JlUm90YXRlZC5zaXplKCldKi87XHJcbiAgICB2YXIgdG90YWxTdW0gPSAwO1xyXG4gICAgdmFyIHN1bU9mUHJvYmFiaWxpdGllcyA9IDA7XHJcbiAgICB2YXIgaSA9IDA7XHJcblxyXG4gICAgdmFyIG51bU9mUHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQgPSBwcm9jZXNzTm9kZXNUb0JlUm90YXRlZC5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBqOyBqPG51bU9mUHJvY2Vzc05vZGVzVG9CZVJvdGF0ZWQ7IGorKylcclxuICAgIHtcclxuICAgICAgICB0b3RhbFN1bSArPSBNYXRoLmFicyhwcm9jZXNzTm9kZXNUb0JlUm90YXRlZFtqXS5uZXRSb3RhdGlvbmFsRm9yY2UpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIG5vcm1hbGl6ZSBhbGwgYmV0d2VlbiAwLi4xXHJcbiAgICB2YXIgbnVtT2ZQcm9jZXNzTm9kZXNUb0JlUm90YXRlZCA9IHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGo7IGo8bnVtT2ZQcm9jZXNzTm9kZXNUb0JlUm90YXRlZDsgaisrKVxyXG4gICAge1xyXG4gICAgICAgIGZpdG5lc3NWYWx1ZXNbaV0gPSBcclxuICAgICAgICAgICAgICAgIHN1bU9mUHJvYmFiaWxpdGllcyArIFxyXG4gICAgICAgICAgICAgICAgKE1hdGguYWJzKHByb2Nlc3NOb2Rlc1RvQmVSb3RhdGVkW2pdLm5ldFJvdGF0aW9uYWxGb3JjZSkgLyBcclxuICAgICAgICAgICAgICAgICAgICB0b3RhbFN1bSk7XHJcblxyXG4gICAgICAgIHN1bU9mUHJvYmFiaWxpdGllcyA9IGZpdG5lc3NWYWx1ZXNbaV07XHJcbiAgICAgICAgaSsrO1xyXG4gICAgfVxyXG4gICAgICAgIFxyXG4gICAgaWYgKHJhbmRvbU51bWJlciA8IGZpdG5lc3NWYWx1ZXNbMF0pXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIDA7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgZm9yICh2YXIgaiA9IDA7IGogPCBmaXRuZXNzVmFsdWVzLmxlbmd0aCAtIDE7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICgocmFuZG9tTnVtYmVyID49IGZpdG5lc3NWYWx1ZXNbal0pICYmIFxyXG4gICAgICAgICAgICAgICAgKHJhbmRvbU51bWJlciA8IGZpdG5lc3NWYWx1ZXNbaisxXSkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiBqICsgMTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gLTE7XHJcbn07XHJcblxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmNhbGNFZmZlY3RvckFuZ2xlID0gZnVuY3Rpb24gKFxyXG4gICAgICAgIC8qT3JpZW50YXRpb24qLyBvcmllbnQsIFxyXG4gICAgICAgIC8qUG9pbnREKi8gICAgICBjZW50ZXJQdCxcclxuICAgICAgICAvKkNvU0VOb2RlKi8gICAgZWZmKVxyXG57XHJcbiAgICB2YXIgaWRlYWxFZGdlTGVuZ3RoID0gdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICB2YXIgdGFyZ2V0UG50ID0gbmV3IFBvaW50RCgpO1xyXG4gICAgdmFyIGNlbnRlclBudCA9IGNlbnRlclB0O1xyXG5cclxuICAgIC8vIGZpbmQgdGFyZ2V0IHBvaW50XHJcbiAgICBpZiAob3JpZW50ID09PSBTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQgfHwgXHJcbiAgICAgICAgb3JpZW50ID09PSBTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UG50LnggPSBjZW50ZXJQbnQueDtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJZKCkgPiBjZW50ZXJQbnQueSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRhcmdldFBudC55ID0gY2VudGVyUG50LnkgKyBpZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRhcmdldFBudC55ID0gY2VudGVyUG50LnkgLSBpZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAob3JpZW50ID09PSBTYmduUERMYXlvdXQuT3JpZW50YXRpb25FbnVtLkJPVFRPTV9UT19UT1AgfHwgXHJcbiAgICAgICAgICAgICBvcmllbnQgPT09IFNiZ25QRExheW91dC5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTSlcclxuICAgIHtcclxuICAgICAgICB0YXJnZXRQbnQueSA9IGNlbnRlclBudC55O1xyXG5cclxuICAgICAgICBpZiAoZWZmLmdldENlbnRlclgoKSA+IGNlbnRlclBudC54KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnggPSBjZW50ZXJQbnQueCArIGlkZWFsRWRnZUxlbmd0aDtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnggPSBjZW50ZXJQbnQueCAtIGlkZWFsRWRnZUxlbmd0aDtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgdmFyIGFuZ2xlID0gSUdlb21ldHJ5LmNhbGN1bGF0ZUFuZ2xlKHRhcmdldFBudCwgY2VudGVyUG50LCBlZmYuZ2V0Q2VudGVyKCkpO1xyXG5cclxuICAgIHJldHVybiBhbmdsZTtcclxufTtcclxuXHJcbi8qKlxyXG4qIFJlY3Vyc2l2ZWx5IGNhbGN1bGF0ZSBpZiB0aGUgbm9kZSBvciBpdHMgY2hpbGQgbm9kZXMgaGF2ZSBhbnkgZWRnZXMgdG9cclxuKiBvdGhlciBub2Rlcy4gUmV0dXJuIHRoZSB0b3RhbCBudW1iZXIgb2YgZWRnZXMuXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY0dyYXBoRGVncmVlID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIHBhcmVudE5vZGUpXHJcbntcclxuICAgIHZhciBkZWdyZWUgPSAwO1xyXG4gICAgaWYgKHBhcmVudE5vZGUuZ2V0Q2hpbGQoKSA9PSBudWxsKVxyXG4gICAge1xyXG4gICAgICAgIGRlZ3JlZSA9IHBhcmVudE5vZGUuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgcmV0dXJuIGRlZ3JlZTtcclxuICAgIH1cclxuXHJcbiAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IHBhcmVudE5vZGUuZ2V0Q2hpbGQoKS5nZXROb2RlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNoaWxkcmVuOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgZGVncmVlID0gZGVncmVlICsgcGFyZW50Tm9kZS5nZXRFZGdlcygpLmxlbmd0aFxyXG4gICAgICAgICAgICAgICAgICAgICAgICArIHRoaXMuY2FsY0dyYXBoRGVncmVlKHBhcmVudE5vZGUuZ2V0Q2hpbGQoKS5nZXROb2RlcygpW2ldKTtcclxuICAgIH1cclxuICAgIFxyXG4gICAgcmV0dXJuIGRlZ3JlZTtcclxufTtcclxuXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUucmVjYWxjUHJvcGVybHlPcmllbnRlZEVkZ2VzID0gZnVuY3Rpb24gKC8qYm9vbGVhbiovIGlzTGFzdEl0ZXJhdGlvbilcclxue1xyXG4gICAgdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50ID0gMC4wO1xyXG4gICAgdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZCA9IDA7XHJcbiAgICBcclxuICAgIHZhciBudW1PZlByb2Nlc3NOb2RlcyA9IHRoaXMucHJvY2Vzc05vZGVMaXN0Lmxlbmd0aDtcclxuICAgIGZvcih2YXIgaT0wOyBpPG51bU9mUHJvY2Vzc05vZGVzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHNiZ25Qcm9jZXNzTm9kZSA9IHRoaXMucHJvY2Vzc05vZGVMaXN0W2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5jYWxjUHJvcGVybHlPcmllbnRlZEVkZ2VzKCk7XHJcbiAgICAgICAgdGhpcy5wcm9wZXJseU9yaWVudGVkRWRnZUNvdW50ICs9IHNiZ25Qcm9jZXNzTm9kZS5wcm9wZXJFZGdlQ291bnQ7XHJcbiAgICAgICAgdGhpcy50b3RhbEVkZ2VDb3VudFRvQmVPcmllbnRlZCArPSBcclxuICAgICAgICAgICAgICAgIChzYmduUHJvY2Vzc05vZGUuY29uc3VtcHRpb25FZGdlcy5sZW5ndGggKyBcclxuICAgICAgICAgICAgICAgICBzYmduUHJvY2Vzc05vZGUucHJvZHVjdEVkZ2VzLmxlbmd0aCArIFxyXG4gICAgICAgICAgICAgICAgIHNiZ25Qcm9jZXNzTm9kZS5lZmZlY3RvckVkZ2VzLmxlbmd0aCk7XHJcbiAgICAgICAgdGhpcy5zdWNjZXNzUmF0aW8gPSBcclxuICAgICAgICAgICAgICAgIHRoaXMucHJvcGVybHlPcmllbnRlZEVkZ2VDb3VudCAvIHRoaXMudG90YWxFZGdlQ291bnRUb0JlT3JpZW50ZWQ7XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBmaW5kcyBhbGwgdGhlIHplcm8gZGVncmVlIG5vZGVzIGluIHRoZSBncmFwaCB3aGljaCBhcmUgbm90XHJcbiogb3duZWQgYnkgYSBjb21wbGV4IG5vZGUuIFplcm8gZGVncmVlIG5vZGVzIGF0IGVhY2ggbGV2ZWwgYXJlIGdyb3VwZWRcclxuKiB0b2dldGhlciBhbmQgcGxhY2VkIGluc2lkZSBhIGR1bW15IGNvbXBsZXggdG8gcmVkdWNlIGJvdW5kcyBvZiByb290XHJcbiogZ3JhcGguXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuZ3JvdXBaZXJvRGVncmVlTWVtYmVycyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBjaGlsZENvbXBsZXhNYXAgPSBuZXcgSGFzaE1hcCgpOy8qU2JnblBETm9kZSwgTEdyYXBoKi9cclxuICAgIFxyXG4gICAgdmFyIG51bU9mR3JhcGhzID0gdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZHcmFwaHM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgb3duZXJHcmFwaCA9IHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKClbaV07XHJcbiAgICAgICAgdmFyIHplcm9EZWdyZWVOb2RlcyA9IFtdOyAvKkFycmF5TGlzdDxTYmduUEROb2RlPiovXHJcbiAgICAgICAgXHJcbiAgICAgICAgLy8gZG8gbm90IHByb2Nlc3MgY29tcGxleCBub2RlcyAodGhlaXIgbWVtYmVycyBhcmUgYWxyZWFkeSBvd25lZClcclxuICAgICAgICBpZiAoKG93bmVyR3JhcGguZ2V0UGFyZW50KCkudHlwZSAhPT0gbnVsbCkgJiYgXHJcbiAgICAgICAgICAgIChvd25lckdyYXBoLmdldFBhcmVudCgpLmlzQ29tcGxleCgpKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuICAgICAgICBcclxuICAgICAgICB2YXIgbnVtT2ZOb2RlcyA9IG93bmVyR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgaj0wOyBqPG51bU9mTm9kZXM7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBub2RlID0gb3duZXJHcmFwaC5nZXROb2RlcygpW2pdO1xyXG5cclxuICAgICAgICAgICAgaWYgKHRoaXMuY2FsY0dyYXBoRGVncmVlKG5vZGUpID09PSAwKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB6ZXJvRGVncmVlTm9kZXMucHVzaChub2RlKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgaWYgKHplcm9EZWdyZWVOb2Rlcy5sZW5ndGggPiAxKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgLy8gY3JlYXRlIGEgbmV3IGR1bW15IGNvbXBsZXhcclxuICAgICAgICAgICAgdmFyIGNvbXBsZXggPSB0aGlzLm5ld05vZGUobnVsbCk7XHJcbiAgICAgICAgICAgIGNvbXBsZXgudHlwZSA9IFNiZ25QRENvbnN0YW50cy5DT01QTEVYO1xyXG4gICAgICAgICAgICBjb21wbGV4LmxhYmVsID0gXCJEdW1teUNvbXBsZXhfXCIgKyBvd25lckdyYXBoLmdldFBhcmVudCgpLmxhYmVsO1xyXG5cclxuICAgICAgICAgICAgb3duZXJHcmFwaC5hZGQoY29tcGxleCk7XHJcblxyXG4gICAgICAgICAgICB2YXIgY2hpbGRHcmFwaCA9IG5ld0dyYXBoKG51bGwpO1xyXG4gICAgICAgICAgICBcclxuICAgICAgICAgICAgdmFyIG51bU9mWmVyb0RlZ3JlZU5vZGUgPSB6ZXJvRGVncmVlTm9kZXMubGVuZ3RoO1xyXG4gICAgICAgICAgICBmb3IgKHZhciBqPTA7IGo8bnVtT2ZaZXJvRGVncmVlTm9kZTsgaisrKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB2YXIgemVyb05vZGUgPSB6ZXJvRGVncmVlTm9kZXNbal07XHJcbiAgICAgICAgICAgICAgICBvd25lckdyYXBoLnJlbW92ZSh6ZXJvTm9kZSk7XHJcbiAgICAgICAgICAgICAgICBjaGlsZEdyYXBoLmFkZCh6ZXJvTm9kZSk7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIHRoaXMuZHVtbXlDb21wbGV4TGlzdC5wdXNoKGNvbXBsZXgpO1xyXG4gICAgICAgICAgICBjaGlsZENvbXBsZXhNYXAucHV0KGNvbXBsZXgsIGNoaWxkR3JhcGgpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIFxyXG4gICAgdmFyIG51bU9mQ29tcGxleE5vZGVzID0gdGhpcy5kdW1teUNvbXBsZXhMaXN0Lmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNvbXBsZXhOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLmFkZChjaGlsZENvbXBsZXhNYXAuZ2V0KGNvbXBsZXgpLCB0aGlzLmR1bW15Q29tcGxleExpc3RbaV0pO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnVwZGF0ZUJvdW5kcygpO1xyXG5cclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24oKTtcclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsRWRnZXMoKTtcclxuICAgIHRoaXMuY2FsY3VsYXRlTm9kZXNUb0FwcGx5R3Jhdml0YXRpb25UbygpO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY3JlYXRlcyB0d28gcG9ydCBub2RlcyBhbmQgYSBjb21wb3VuZCBmb3IgZWFjaCBwcm9jZXNzIG5vZGVzXHJcbiogYW5kIGFkZHMgdGhlbSB0byBncmFwaC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jcmVhdGVQb3J0Tm9kZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgbnVtT2ZOb2RlcyA9IHRoaXMuZ2V0QWxsTm9kZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBvcmlnaW5hbFByb2Nlc3NOb2RlID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAob3JpZ2luYWxQcm9jZXNzTm9kZS50eXBlID09PSBTYmduUERDb25zdGFudHMuUFJPQ0VTUylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBvd25lckdyYXBoID0gb3JpZ2luYWxQcm9jZXNzTm9kZS5nZXRPd25lcigpO1xyXG5cclxuICAgICAgICAgICAgLy8gY3JlYXRlIG5ldyBub2RlcyBhbmQgZ3JhcGhzXHJcbiAgICAgICAgICAgIHZhciBwcm9jZXNzTm9kZSA9IG5ld1Byb2Nlc3NOb2RlKG51bGwpO1xyXG4gICAgICAgICAgICB2YXIgaW5wdXRQb3J0ICAgPSBuZXdQb3J0Tm9kZShudWxsLCBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVCk7XHJcbiAgICAgICAgICAgIHZhciBvdXRwdXRQb3J0ICA9IG5ld1BvcnROb2RlKG51bGwsIFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVCk7XHJcblxyXG4gICAgICAgICAgICAvLyBjcmVhdGUgYSBkdW1teSBjb21wb3VuZFxyXG4gICAgICAgICAgICB2YXIgY29tcG91bmROb2RlID0gbmV3Tm9kZShudWxsKTtcclxuICAgICAgICAgICAgY29tcG91bmROb2RlLnR5cGUgPSBTYmduUERDb25zdGFudHMuRFVNTVlfQ09NUE9VTkQ7XHJcblxyXG4gICAgICAgICAgICAvLyBhZGQgbGFiZWxzXHJcbiAgICAgICAgICAgIGNvbXBvdW5kTm9kZS5sYWJlbCA9IFwiRHVtbXlDb21wb3VuZF9cIiArIG9yaWdpbmFsUHJvY2Vzc05vZGUubGFiZWw7XHJcbiAgICAgICAgICAgIGlucHV0UG9ydC5sYWJlbCA9IFwiSW5wdXRQb3J0X1wiICsgb3JpZ2luYWxQcm9jZXNzTm9kZS5sYWJlbDtcclxuICAgICAgICAgICAgb3V0cHV0UG9ydC5sYWJlbCA9IFwiT3V0cHV0UG9ydF9cIiArIG9yaWdpbmFsUHJvY2Vzc05vZGUubGFiZWw7XHJcblxyXG4gICAgICAgICAgICAvLyBjcmVhdGUgY2hpbGQgZ3JhcGggKD0gMnBvcnQrcHJvY2VzcykgdG8gYmUgc2V0IGFzIGNoaWxkIHRvXHJcbiAgICAgICAgICAgIC8vIGR1bW15IGNvbXBvdW5kXHJcbiAgICAgICAgICAgIHZhciBjaGlsZEdyYXBoID0gbmV3R3JhcGgobnVsbCk7XHJcbiAgICAgICAgICAgIG93bmVyR3JhcGguYWRkKHByb2Nlc3NOb2RlKTtcclxuXHJcbiAgICAgICAgICAgIC8vIGNvbnZlcnQgdGhlIHByb2Nlc3Mgbm9kZSB0byBTYmduUHJvY2Vzc05vZGVcclxuICAgICAgICAgICAgcHJvY2Vzc05vZGUuY29weUZyb21TQkdOUEROb2RlKG9yaWdpbmFsUHJvY2Vzc05vZGUsXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpKTtcclxuXHJcbiAgICAgICAgICAgIHByb2Nlc3NOb2RlLmNvbm5lY3ROb2Rlcyhjb21wb3VuZE5vZGUsIGlucHV0UG9ydCwgb3V0cHV0UG9ydCk7XHJcblxyXG4gICAgICAgICAgICAvLyBjcmVhdGUgcmlnaWQgZWRnZXMsIGNoYW5nZSBlZGdlIGNvbm5lY3Rpb25zXHJcbiAgICAgICAgICAgIHByb2Nlc3NOb2RlLnJlY29ubmVjdEVkZ2VzKGlkZWFsRWRnZUxlbmd0aCk7XHJcblxyXG4gICAgICAgICAgICB2YXIgcmlnaWRUb1Byb2R1Y3Rpb24gPSBuZXdSaWdpZEVkZ2UobnVsbCk7XHJcbiAgICAgICAgICAgIHJpZ2lkVG9Qcm9kdWN0aW9uLmxhYmVsID0gXCJcIlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKyAodGhpcy5ncmFwaE1hbmFnZXIuZ2V0QWxsRWRnZXMoKS5sZW5ndGggKyAxKTtcclxuXHJcbiAgICAgICAgICAgIHZhciByaWdpZFRvQ29uc3VtcHRpb24gPSBuZXdSaWdpZEVkZ2UobnVsbCk7XHJcbiAgICAgICAgICAgIHJpZ2lkVG9Db25zdW1wdGlvbi5sYWJlbCA9IFwiXCJcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICsgKHRoaXMuZ3JhcGhNYW5hZ2VyLmdldEFsbEVkZ2VzKCkubGVuZ3RoICsgMik7XHJcblxyXG4gICAgICAgICAgICBvd25lckdyYXBoLnJlbW92ZShwcm9jZXNzTm9kZSk7XHJcblxyXG4gICAgICAgICAgICAvLyBvcmdhbml6ZSBjaGlsZCBncmFwaFxyXG4gICAgICAgICAgICBjaGlsZEdyYXBoLmFkZChwcm9jZXNzTm9kZSk7XHJcbiAgICAgICAgICAgIGNoaWxkR3JhcGguYWRkKGlucHV0UG9ydCk7XHJcbiAgICAgICAgICAgIGNoaWxkR3JhcGguYWRkKG91dHB1dFBvcnQpO1xyXG4gICAgICAgICAgICBjaGlsZEdyYXBoLmFkZChyaWdpZFRvUHJvZHVjdGlvbiwgaW5wdXRQb3J0LCBwcm9jZXNzTm9kZSk7XHJcbiAgICAgICAgICAgIGNoaWxkR3JhcGguYWRkKHJpZ2lkVG9Db25zdW1wdGlvbiwgb3V0cHV0UG9ydCwgcHJvY2Vzc05vZGUpO1xyXG5cclxuICAgICAgICAgICAgLy8gb3JnYW5pemUgdGhlIGNvbXBvdW5kIG5vZGVcclxuICAgICAgICAgICAgY29tcG91bmROb2RlLnNldE93bmVyKG93bmVyR3JhcGgpO1xyXG4gICAgICAgICAgICBjb21wb3VuZE5vZGUuc2V0Q2VudGVyKHByb2Nlc3NOb2RlLmdldENlbnRlclgoKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHByb2Nlc3NOb2RlLmdldENlbnRlclkoKSk7XHJcbiAgICAgICAgICAgIG93bmVyR3JhcGguYWRkKGNvbXBvdW5kTm9kZSk7XHJcbiAgICAgICAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLmFkZChjaGlsZEdyYXBoLCBjb21wb3VuZE5vZGUpO1xyXG5cclxuICAgICAgICAgICAgLy8gcmVtb3ZlIHRoZSBvcmlnaW5hbCBwcm9jZXNzIG5vZGVcclxuICAgICAgICAgICAgb3duZXJHcmFwaC5yZW1vdmUob3JpZ2luYWxQcm9jZXNzTm9kZSk7XHJcblxyXG4gICAgICAgICAgICB0aGlzLnByb2Nlc3NOb2RlTGlzdC5wdXNoKHByb2Nlc3NOb2RlKTtcclxuICAgICAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIudXBkYXRlQm91bmRzKCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIC8vIHJlc2V0IHRoZSB0b3BvbG9neVxyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxOb2RlcygpO1xyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbigpO1xyXG4gICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxFZGdlcygpO1xyXG5cclxuICAgIHRoaXMuY2FsY3VsYXRlTm9kZXNUb0FwcGx5R3Jhdml0YXRpb25UbygpO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY2hlY2tzIHdoZXRoZXIgdGhlcmUgZXhpc3RzIGFueSBwcm9jZXNzIG5vZGVzIGluIHRoZSBncmFwaC5cclxuKiBJZiB0aGVyZSBleGlzdCBhbnkgcHJvY2VzcyBub2RlcyBpdCBpcyBhc3N1bWVkIHRoYXQgdGhlIGdpdmVuIGdyYXBoXHJcbiogcmVzcGVjdHMgb3VyIHN0cnVjdHVyZS5cclxuKiBcclxuKiBNb3N0IGxpa2VseTogdGhpcyBtZXRob2QgZG9lcyBub3Qgd29yayBwcm9wZXJseS4gTmV2ZXIgaGFkIGFueSBpbnB1dCB0b1xyXG4qIHRlc3QuIE5vdCBjb21wbGV0ZS5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5hcmVQb3J0Tm9kZXNDcmVhdGVkID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGZsYWcgPSBmYWxzZTtcclxuXHJcbiAgICAvLyBpZiB0aGVyZSBhcmUgYW55IHByb2Nlc3Mgbm9kZXMsIGNoZWNrIGZvciBwb3J0IG5vZGVzXHJcbiAgICB2YXIgbnVtT2ZOb2RlcyA9IHRoaXMuZ2V0QWxsTm9kZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZOb2RlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzYmduUEROb2RlID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIGlmIChzYmduUEROb2RlLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5QUk9DRVNTKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgZmxhZyA9IHRydWU7XHJcbiAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIFxyXG4gICAgLy8gaWYgdGhlcmUgYXJlIG5vIHByb2Nlc3Mgbm9kZXMsIG5vIG5lZWQgdG8gY2hlY2sgZm9yIHBvcnQgbm9kZXNcclxuICAgIGlmICghZmxhZylcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICAvLyBjaGVjayBmb3IgdGhlIHBvcnQgbm9kZXMuIGlmIGFueSBmb3VuZCwgcmV0dXJuIHRydWUuXHJcbiAgICAgICAgdmFyIG51bU9mTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGk9MDsgaTxudW1PZk5vZGVzOyBpKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgc2JnblBETm9kZSA9IHRoaXMuZ2V0QWxsTm9kZXMoKVtpXTtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIGlmIChzYmduUEROb2RlLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgc2JnblBETm9kZS50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiB0cnVlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgXHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBpcyB1c2VkIHRvIHJlbW92ZSB0aGUgZHVtbXkgY29tcG91bmRzIChwcmV2aW91c2x5IGNyZWF0ZWQgZm9yXHJcbiogZWFjaCBwcm9jZXNzIG5vZGUpIGZyb20gdGhlIGdyYXBoLlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLnJlbW92ZUR1bW15Q29tcG91bmRzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIG51bU9mUHJvY2Vzc05vZGVzID0gdGhpcy5wcm9jZXNzTm9kZUxpc3QubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaT0wOyBpPG51bU9mUHJvY2Vzc05vZGVzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHByb2Nlc3NOb2RlID0gdGhpcy5wcm9jZXNzTm9kZUxpc3RbaV07XHJcbiAgICAgICAgdmFyIGR1bW15Tm9kZSA9IHByb2Nlc3NOb2RlLnBhcmVudENvbXBvdW5kO1xyXG4gICAgICAgIHZhciBjaGlsZEdyYXBoID0gZHVtbXlOb2RlLmdldENoaWxkKCk7XHJcbiAgICAgICAgdmFyIG93bmVyID0gZHVtbXlOb2RlLmdldE93bmVyKCk7XHJcblxyXG4gICAgICAgIC8vIGFkZCBjaGlsZHJlbiB0byBvcmlnaW5hbCBwYXJlbnRcclxuICAgICAgICB2YXIgbnVtT2ZDaGlsZE5vZGVzID0gY2hpbGRHcmFwaC5nZXROb2RlcygpLmxlbmd0aDtcclxuICAgICAgICBmb3IgKHZhciBqPTA7IGo8bnVtT2ZDaGlsZE5vZGVzOyBqKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBvd25lci5hZGQoY2hpbGRHcmFwaC5nZXROb2RlcygpW2pdKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHZhciBudW1PZkNoaWxkRWRnZXMgPSBjaGlsZEdyYXBoLmdldEVkZ2VzKCkubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGo9MDsgajxudW1PZkNoaWxkRWRnZXM7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBlZGdlID0gY2hpbGRHcmFwaC5nZXRFZGdlcygpW2pdXHJcbiAgICAgICAgICAgIG93bmVyLmFkZChlZGdlLCBlZGdlLmdldFNvdXJjZSgpLCBlZGdlLmdldFRhcmdldCgpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgXHJcbiAgICAgICAgLy8gYWRkIGVmZmVjdG9ycyAvIHJlbWFpbmluZyBlZGdlcyBiYWNrIHRvIHRoZSBwcm9jZXNzXHJcbiAgICAgICAgZm9yICh2YXIgaiA9IDA7IGogPCBkdW1teU5vZGUuZ2V0RWRnZXMoKS5sZW5ndGg7IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBlZGdlID0gZHVtbXlOb2RlLmdldEVkZ2VzKClbal07XHJcblxyXG4gICAgICAgICAgICBkdW1teU5vZGUuZ2V0RWRnZXMoKS5zcGxpY2UoaiwgMSk7XHJcbiAgICAgICAgICAgIGVkZ2Uuc2V0VGFyZ2V0KHByb2Nlc3NOb2RlKTtcclxuICAgICAgICAgICAgcHJvY2Vzc05vZGUuZ2V0RWRnZXMoKS5wdXNoKGVkZ2UpO1xyXG4gICAgICAgICAgICBqLS07XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyByZW1vdmUgdGhlIGdyYXBoXHJcbiAgICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5zcGxpY2UodGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5pbmRleE9mKGNoaWxkR3JhcGgpLCAxKTtcclxuICAgICAgICBkdW1teU5vZGUuc2V0Q2hpbGQobnVsbCk7XHJcbiAgICAgICAgb3duZXIucmVtb3ZlKGR1bW15Tm9kZSk7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5yZXNldEFsbE5vZGVzKCk7XHJcbiAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnJlc2V0QWxsTm9kZXNUb0FwcGx5R3Jhdml0YXRpb24oKTtcclxuICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkucmVzZXRBbGxFZGdlcygpO1xyXG4gICAgdGhpcy5jYWxjdWxhdGVOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvblRvKCk7XHJcbn07XHJcblxyXG5cclxuLy8gKioqKioqKioqKioqKioqKioqKioqIFNFQ1RJT04gOiBUSUxJTkcgTUVUSE9EUyAqKioqKioqKioqKioqKioqKioqKipcclxuXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2xlYXJDb21wbGV4ID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIGNvbXApXHJcbntcclxuICAgIHZhciBwYWNrID0gbnVsbDsgLyogTWVtYmVyUGFjayAqL1xyXG4gICAgdmFyIGNoaWxkR3IgPSBjb21wLmdldENoaWxkKCk7IC8qIExHcmFwaCAqL1xyXG4gICAgdGhpcy5jaGlsZEdyYXBoTWFwLnB1dChjb21wLCBjaGlsZEdyKTtcclxuXHJcbiAgICBpZiAoY2hpbGRHciA9PSBudWxsKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybjtcclxuICAgIH1cclxuICAgIFxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PSBTYmduUERMYXlvdXQuRGVmYXVsdENvbXBhY3Rpb25BbGdvcml0aG1FbnVtLlBPTFlPTUlOT19QQUNLSU5HKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuYXBwbHlQb2x5b21pbm8oY29tcCk7XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmICh0aGlzLmNvbXBhY3Rpb25NZXRob2QgPT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5USUxJTkcpXHJcbiAgICB7XHJcbiAgICAgICAgcGFjayA9IG5ldyBNZW1iZXJQYWNrKGNoaWxkR3IpO1xyXG4gICAgICAgIHRoaXMubWVtYmVyUGFja01hcC5wdXQoY29tcCwgcGFjayk7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHRoaXMuZHVtbXlDb21wbGV4TGlzdC5pbmNsdWRlcyhjb21wKSlcclxuICAgIHtcclxuICAgICAgICBmb3IgKHZhciBpPTA7IGk8Y29tcC5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoOyBpKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBjbGVhckR1bW15Q29tcGxleEdyYXBocyhjb21wLmdldENoaWxkKCkuZ2V0Tm9kZXMoKVtpXSk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHZhciByZW1JbmRleCA9IHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkuaW5kZXhPZihjaGlsZEdyKTtcclxuICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkuc3BsaWNlKHJlbUluZGV4LCAxKTtcclxuICAgIGNvbXAuc2V0Q2hpbGQobnVsbCk7XHJcblxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PSBTYmduUERMYXlvdXQuRGVmYXVsdENvbXBhY3Rpb25BbGdvcml0aG1FbnVtLlRJTElORylcclxuICAgIHtcclxuICAgICAgICBjb21wLnNldFdpZHRoKHBhY2suZ2V0V2lkdGgoKSk7XHJcbiAgICAgICAgY29tcC5zZXRIZWlnaHQocGFjay5nZXRIZWlnaHQoKSk7XHJcbiAgICB9XHJcblxyXG4gICAgLy8gUmVkaXJlY3QgdGhlIGVkZ2VzIG9mIGNvbXBsZXggbWVtYmVycyB0byB0aGUgY29tcGxleC5cclxuICAgIGlmIChjaGlsZEdyICE9IG51bGwpXHJcbiAgICB7XHJcbiAgICAgICAgZm9yICh2YXIgaT0wOyBpPGNoaWxkR3IuZ2V0Tm9kZXMoKS5sZW5ndGg7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBjaE5kID0gY2hpbGRHci5nZXROb2RlcygpW2ldO1xyXG5cclxuICAgICAgICAgICAgZm9yICh2YXIgaj0wOyBqPGNoTmQuZ2V0RWRnZXMoKS5sZW5ndGg7IGorKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdmFyIGVkZ2UgPSBjaE5kLmdldEVkZ2VzKClbal07XHJcbiAgICAgICAgICAgICAgICBpZiAoZWRnZS5nZXRTb3VyY2UoKSA9PSBjaE5kKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGNoTmQuZ2V0RWRnZXMoKS5zcGxpY2UoY2hOZC5nZXRFZGdlcygpLmluZGV4T2YoZWRnZSksIDEpO1xyXG4gICAgICAgICAgICAgICAgICAgIGVkZ2Uuc2V0U291cmNlKGNvbXApO1xyXG4gICAgICAgICAgICAgICAgICAgIGNvbXAuZ2V0RWRnZXMoKS5wdXNoKGVkZ2UpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgZWxzZSBpZiAoZWRnZS5nZXRUYXJnZXQoKSA9PSBjaE5kKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGNoTmQuZ2V0RWRnZXMoKS5zcGxpY2UoY2hOZC5nZXRFZGdlcygpLmluZGV4T2YoZWRnZSksIDEpO1xyXG4gICAgICAgICAgICAgICAgICAgIGVkZ2Uuc2V0VGFyZ2V0KGNvbXApO1xyXG4gICAgICAgICAgICAgICAgICAgIGNvbXAuZ2V0RWRnZXMoKS5wdXNoKGVkZ2UpO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2Qgc2VhcmNoZWQgdW5tYXJrZWQgY29tcGxleCBub2RlcyByZWN1cnNpdmVseSwgYmVjYXVzZSB0aGV5IG1heVxyXG4qIGNvbnRhaW4gY29tcGxleCBjaGlsZHJlbi4gQWZ0ZXIgdGhlIG9yZGVyIGlzIGZvdW5kLCBjaGlsZCBncmFwaHMgb2YgZWFjaFxyXG4qIGNvbXBsZXggbm9kZSBhcmUgY2xlYXJlZC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5hcHBseURGU09uQ29tcGxleGVzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgICAgLy8gTEdyYXBoPigpO1xyXG4gICAgICAgdmFyIG51bU9mTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBudW1PZk5vZGVzOyBpKyspXHJcbiAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBub2RlID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG4gICAgICAgICAgIFxyXG4gICAgICAgICAgICAvLyBUT0RPOiBJbnN0YW5jZSBvZiFcclxuICAgICAgICAgICAgaWYgKG5vZGUuaXNDb21wbGV4KCkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIC8vIGNvbXBsZXggaXMgZm91bmQsIHJlY3Vyc2Ugb24gaXQgdW50aWwgbm8gdmlzaXRlZCBjb21wbGV4IHJlbWFpbnMuXHJcbiAgICAgICAgICAgIGlmICghbm9kZS52aXNpdGVkKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB0aGlzLkRGU1Zpc2l0Q29tcGxleChub2RlKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgfVxyXG5cclxuICAgICAgIC8vIGNsZWFyIGVhY2ggY29tcGxleFxyXG4gICAgICAgdmFyIG51bU9mQ29tcGxleE9yZGVyID0gdGhpcy5jb21wbGV4T3JkZXIubGVuZ3RoO1xyXG4gICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBudW1PZkNvbXBsZXhPcmRlcjsgaSsrKVxyXG4gICAgICAge1xyXG4gICAgICAgICAgIGNsZWFyQ29tcGxleCh0aGlzLmNvbXBsZXhPcmRlcltpXSk7XHJcbiAgICAgICB9XHJcblxyXG4gICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS51cGRhdGVCb3VuZHMoKTtcclxuXHJcbiAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbigpO1xyXG4gICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5yZXNldEFsbEVkZ2VzKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCByZWN1cnNlcyBvbiB0aGUgY29tcGxleCBvYmplY3RzLiBJZiBhIG5vZGUgZG9lcyBub3QgY29udGFpblxyXG4qIGFueSBjb21wbGV4IG5vZGVzIG9yIGFsbCB0aGUgbm9kZXMgaW4gdGhlIGNoaWxkIGdyYXBoIGlzIGFscmVhZHkgbWFya2VkLFxyXG4qIGl0IGlzIHJlcG9ydGVkLiAoRGVwdGggZmlyc3QpXHJcbiogXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuREZTVmlzaXRDb21wbGV4ID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIG5vZGUpXHJcbntcclxuICAgIGlmIChub2RlLmdldENoaWxkKCkgIT0gbnVsbClcclxuICAgIHtcclxuICAgICAgICAgdmFyIG51bU9mQ2hpbGRyZW4gPSAgbm9kZS5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IG51bU9mQ2hpbGRyZW47IGkrKylcclxuICAgICAgICAge1xyXG4gICAgICAgICAgICAgdGhpcy5ERlNWaXNpdENvbXBsZXgobm9kZS5nZXRDaGlsZCgpLmdldE5vZGVzKClbaV0pO1xyXG4gICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKG5vZGUuaXNDb21wbGV4KCkgJiYgIW5vZGUuY29udGFpbnNVbm1hcmtlZENvbXBsZXgoKSlcclxuICAgIHtcclxuICAgICAgICAgdGhpcy5jb21wbGV4T3JkZXIucHVzaChub2RlKTtcclxuICAgICAgICAgbm9kZS52aXNpdGVkID0gdHJ1ZTtcclxuICAgICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgdGlsZXMgdGhlIGdpdmVuIGxpc3Qgb2Ygbm9kZXMgYnkgdXNpbmcgcG9seW9taW5vIHBhY2tpbmdcclxuKiBhbGdvcml0aG0uXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuYXBwbHlQb2x5b21pbm8gPSBmdW5jdGlvbiAoLypTYmduUEROb2RlKi8gcGFyZW50KVxyXG57XHJcbiAgICB2YXIgcmVjdDtcclxuICAgIHZhciBjaGlsZEdyID0gcGFyZW50LmdldENoaWxkKCk7XHJcblxyXG4gICAgaWYgKGNoaWxkR3IgPT0gbnVsbClcclxuICAgIHtcclxuICAgICAgICBjb25zb2xlLmxvZyhcIkNoaWxkIGdyYXBoIGlzIGVtcHR5IChQb2x5b21pbm8pXCIpO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIC8vIHBhY2tpbmcgdGFrZXMgdGhlIGlucHV0IGFzIGFuIGFycmF5LiBwdXQgdGhlIG1lbWJlcnMgaW4gYW4gYXJyYXkuXHJcbiAgICAgICAgdmFyIG1wQXJyYXkgPSBbXTtcclxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IGNoaWxkR3IuZ2V0Tm9kZXMoKS5sZW5ndGg7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG1wQXJyYXlbaV0gPSBjaGlsZEdyLmdldE5vZGVzKClbaV07XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBwYWNrIHJlY3RhbmdsZXNcclxuICAgICAgICBSZWN0UHJvYy5wYWNrUmVjdGFuZ2xlc01pbm8oXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9IT1JJWk9OVEFMX0JVRkZFUixcclxuICAgICAgICAgICAgICAgICAgICAgICAgbXBBcnJheS5sZW5ndGgsIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICBtcEFycmF5KTtcclxuXHJcbiAgICAgICAgLy8gYXBwbHkgY29tcGFjdGlvblxyXG4gICAgICAgIHZhciBjID0gbmV3IENvbXBhY3Rpb24oY2hpbGRHci5nZXROb2RlcygpKTtcclxuICAgICAgICBjLnBlcmZvcm0oKTtcclxuXHJcbiAgICAgICAgLy8gZ2V0IHRoZSByZXN1bHRpbmcgcmVjdGFuZ2xlIGFuZCBzZXQgcGFyZW50J3MgKGNvbXBsZXgpIHdpZHRoICZcclxuICAgICAgICAvLyBoZWlnaHRcclxuICAgICAgICByZWN0ID0gdGhpcy5jYWxjdWxhdGVCb3VuZHModHJ1ZSwgY2hpbGRHci5nZXROb2RlcygpKTtcclxuXHJcbiAgICAgICAgcGFyZW50LnNldFdpZHRoKHJlY3QuZ2V0V2lkdGgoKSk7XHJcbiAgICAgICAgcGFyZW50LnNldEhlaWdodChyZWN0LmdldEhlaWdodCgpKTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFJlYXNzaWducyB0aGUgY29tcGxleCBjb250ZW50LiBUaGUgb3V0ZXJtb3N0IGNvbXBsZXggaXMgcGxhY2VkIGZpcnN0LlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLnJlcG9wdWxhdGVDb21wbGV4ZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgZW1wdGllZER1bW15Q29tcGxleE1hcFNpemUgPSB0aGlzLmVtcHRpZWREdW1teUNvbXBsZXhNYXAua2V5U2V0KCkubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBlbXB0aWVkRHVtbXlDb21wbGV4TWFwU2l6ZTsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBjb21wID0gdGhpcy5lbXB0aWVkRHVtbXlDb21wbGV4TWFwLmtleVNldCgpW2ldO1xyXG4gICAgICAgIHZhciBjaEdyID0gdGhpcy5lbXB0aWVkRHVtbXlDb21wbGV4TWFwLmdldChjb21wKTtcclxuICAgICAgICBjb21wLnNldENoaWxkKGNoR3IpO1xyXG4gICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkucHVzaChjaEdyKTtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKHZhciBpID0gdGhpcy5jb21wbGV4T3JkZXIubGVuZ3RoIC0gMTsgaSA+PSAwOyBpLS0pXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGNvbXAgPSB0aGlzLmNvbXBsZXhPcmRlcltpXTtcclxuICAgICAgICB2YXIgY2hHciA9IHRoaXMuY2hpbGRHcmFwaE1hcC5nZXQoY29tcCk7XHJcblxyXG4gICAgICAgIC8vIHJlcG9wdWxhdGUgdGhlIGNvbXBsZXhcclxuICAgICAgICBjb21wLnNldENoaWxkKGNoR3IpO1xyXG5cclxuICAgICAgICAvLyBpZiB0aGUgY2hpbGQgZ3JhcGggaXMgbm90IG51bGwsIGFkanVzdCB0aGUgcG9zaXRpb25zIG9mIG1lbWJlcnNcclxuICAgICAgICBpZiAoY2hHciAhPSBudWxsKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgLy8gYWRqdXN0IHRoZSBwb3NpdGlvbnMgb2YgdGhlIG1lbWJlcnNcclxuICAgICAgICAgICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5QT0xZT01JTk9fUEFDS0lORylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5hZGp1c3RMb2NhdGlvbihjb21wLCBjaEdyKTtcclxuICAgICAgICAgICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkucHVzaChjaEdyKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlIGlmIChjb21wYWN0aW9uTWV0aG9kID09PSBTYmduUERMYXlvdXQuRGVmYXVsdENvbXBhY3Rpb25BbGdvcml0aG1FbnVtLlRJTElORylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5wdXNoKGNoR3IpO1xyXG5cclxuICAgICAgICAgICAgICAgIHZhciBwYWNrID0gdGhpcy5tZW1iZXJQYWNrTWFwLmdldChjb21wKTtcclxuICAgICAgICAgICAgICAgIHBhY2suYWRqdXN0TG9jYXRpb25zKGNvbXAuZ2V0TGVmdCgpLCBjb21wLmdldFRvcCgpKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIFxyXG4gICAgdmFyIGVtcHRpZWREdW1teUNvbXBsZXhNYXBTaXplID0gdGhpcy5lbXB0aWVkRHVtbXlDb21wbGV4TWFwLmtleVNldCgpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgZW1wdGllZER1bW15Q29tcGxleE1hcFNpemU7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgY29tcCA9IHRoaXMuZW1wdGllZER1bW15Q29tcGxleE1hcC5rZXlTZXQoKVtpXTtcclxuICAgICAgICB2YXIgY2hHciA9IHRoaXMuZW1wdGllZER1bW15Q29tcGxleE1hcC5nZXQoY29tcCk7XHJcblxyXG4gICAgICAgIHRoaXMuYWRqdXN0TG9jYXRpb24oY29tcCwgY2hHcik7XHJcbiAgICB9XHJcblxyXG4gICAgdGhpcy5yZW1vdmVEdW1teUNvbXBsZXhlcygpO1xyXG5cclxuICAgIC8vIHJlc2V0XHJcbiAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkucmVzZXRBbGxOb2Rlc1RvQXBwbHlHcmF2aXRhdGlvbigpO1xyXG4gICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5yZXNldEFsbEVkZ2VzKCk7XHJcbiAgICB0aGlzLmNhbGN1bGF0ZU5vZGVzVG9BcHBseUdyYXZpdGF0aW9uVG8oKTtcclxufTtcclxuXHJcbi8qKlxyXG4qIEFkanVzdCBsb2NhdGlvbnMgb2YgY2hpbGRyZW4gb2YgZ2l2ZW4gY29tcGxleCB3cnQuIHRoZSBsb2NhdGlvbiBvZiB0aGVcclxuKiBjb21wbGV4XHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuYWRqdXN0TG9jYXRpb24gPSBmdW5jdGlvbiAoY29tcCwgY2hHcilcclxue1xyXG4gICAgdmFyIHJlY3QgPSBjYWxjdWxhdGVCb3VuZHMoZmFsc2UsIGNoR3IuZ2V0Tm9kZXMoKSk7XHJcblxyXG4gICAgdmFyIGRpZmZlcmVuY2VYID0gKHJlY3QueCAtIGNvbXAuZ2V0TGVmdCgpKTtcclxuICAgIHZhciBkaWZmZXJlbmNlWSA9IChyZWN0LnkgLSBjb21wLmdldFRvcCgpKTtcclxuXHJcbiAgICAvLyBpZiB0aGUgcGFyZW50IGdyYXBoIGlzIGEgY29tcG91bmQsIGFkZCBjb21wb3VuZCBtYXJnaW5zXHJcbiAgICBpZiAoY29tcC50eXBlICE9PSBTYmduUERDb25zdGFudHMuQ09NUExFWClcclxuICAgIHtcclxuICAgICAgICBkaWZmZXJlbmNlWCAtPSBTYmduUERDb25zdGFudHMuQ09NUE9VTkRfTk9ERV9NQVJHSU47XHJcbiAgICAgICAgZGlmZmVyZW5jZVkgLT0gU2JnblBEQ29uc3RhbnRzLkNPTVBPVU5EX05PREVfTUFSR0lOO1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIGogPSAwOyBqIDwgY2hHci5nZXROb2RlcygpLmxlbmd0aDsgaisrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzID0gY2hHci5nZXROb2Rlc1tqXTtcclxuXHJcbiAgICAgICAgcy5zZXRMb2NhdGlvbihzLmdldExlZnQoKSAtIGRpZmZlcmVuY2VYXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX0hPUklaT05UQUxfQlVGRkVSLCBzLmdldFRvcCgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC0gZGlmZmVyZW5jZVkgKyBTYmduUERDb25zdGFudHMuQ09NUExFWF9NRU1fVkVSVElDQUxfQlVGRkVSKTtcclxuXHJcbiAgICAgICAgaWYgKHMuZ2V0Q2hpbGQoKSAhPT0gbnVsbClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuYWRqdXN0TG9jYXRpb24ocywgcy5nZXRDaGlsZCgpKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBSZWN1cnNpdmVseSByZW1vdmVzIGFsbCBkdW1teSBjb21wbGV4IG5vZGVzIChwcmV2aW91c2x5IGNyZWF0ZWQgdG8gdGlsZVxyXG4qIGdyb3VwIGRlZ3JlZS16ZXJvIG5vZGVzKSBmcm9tIHRoZSBncmFwaC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jbGVhckR1bW15Q29tcGxleEdyYXBocyA9IGZ1bmN0aW9uIChjb21wKVxyXG57XHJcbiAgICBpZiAoY29tcC5nZXRDaGlsZCgpID09IG51bGwgfHwgY29tcC5pc0R1bW15Q29tcG91bmQpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IGNvbXAuZ2V0Q2hpbGQoKS5nZXROb2RlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbnVtT2ZDaGlsZHJlbjsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBjaGlsZE5vZGUgPSBjb21wLmdldENoaWxkKCkuZ2V0Tm9kZXMoKVtpXTtcclxuICAgICAgICBpZiAoY2hpbGROb2RlLmdldENoaWxkKCkgIT0gbnVsbCAmJiBjaGlsZE5vZGUuZ2V0RWRnZXMoKS5sZW5ndGggPT0gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuY2xlYXJEdW1teUNvbXBsZXhHcmFwaHMoY2hpbGROb2RlKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBpZiAodGhpcy5ncmFwaE1hbmFnZXIuZ2V0R3JhcGhzKCkuaW5jbHVkZXMoY29tcC5nZXRDaGlsZCgpKSlcclxuICAgIHtcclxuICAgICAgICBpZiAodGhpcy5jYWxjR3JhcGhEZWdyZWUoY29tcCkgPT09IDApXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLmVtcHRpZWREdW1teUNvbXBsZXhNYXAucHV0KGNvbXAsIGNvbXAuZ2V0Q2hpbGQoKSk7XHJcblxyXG4gICAgICAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLmdldEdyYXBocygpLnNwbGljZShcclxuICAgICAgICAgICAgICAgICAgICB0aGlzLmdldEdyYXBoTWFuYWdlcigpLmdldEdyYXBocygpLmluZGV4T2YoY29tcC5nZXRDaGlsZCgpKSwgMSk7XHJcbiAgICAgICAgICAgIGNvbXAuc2V0Q2hpbGQobnVsbCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogRHVtbXkgY29tcGxleGVzIChwbGFjZWQgaW4gdGhlIFwiZHVtbXlDb21wbGV4TGlzdFwiKSBhcmUgcmVtb3ZlZCBmcm9tIHRoZVxyXG4qIGdyYXBoLlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLnJlbW92ZUR1bW15Q29tcGxleGVzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGR1bW15Q29tcGxleExpc3RTaXplID0gdGhpcy5kdW1teUNvbXBsZXhMaXN0Lmxlbmd0aDtcclxuICAgIC8vIHJlbW92ZSBkdW1teSBjb21wbGV4ZXMgYW5kIGNvbm5lY3QgY2hpbGRyZW4gdG8gb3JpZ2luYWwgcGFyZW50XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8ZHVtbXlDb21wbGV4TGlzdFNpemU7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZHVtbXlDb21wbGV4ID0gdGhpcy5kdW1teUNvbXBsZXhMaXN0W2ldO1xyXG4gICAgICAgIHZhciBjaGlsZEdyYXBoID0gZHVtbXlDb21wbGV4LmdldENoaWxkKCk7XHJcbiAgICAgICAgdmFyIG93bmVyID0gZHVtbXlDb21wbGV4LmdldE93bmVyKCk7XHJcblxyXG4gICAgICAgIHRoaXMuZ2V0R3JhcGhNYW5hZ2VyKCkuZ2V0R3JhcGhzKCkuc3BsaWNlKFxyXG4gICAgICAgICAgICAgICAgdGhpcy5nZXRHcmFwaE1hbmFnZXIoKS5nZXRHcmFwaHMoKS5pbmRleE9mKGNoaWxkR3JhcGgpLCAxKTtcclxuICAgICAgICBkdW1teUNvbXBsZXguc2V0Q2hpbGQobnVsbCk7XHJcblxyXG4gICAgICAgIG93bmVyLnJlbW92ZShkdW1teUNvbXBsZXgpO1xyXG5cclxuICAgICAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IGNoaWxkR3JhcGguZ2V0Tm9kZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgaj0wOyBqPG51bU9mQ2hpbGRyZW47IGorKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG93bmVyLmFkZChjaGlsZEdyYXBoLmdldE5vZGVzKClbal0pO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIHJldHVybnMgdGhlIGJvdW5kaW5nIHJlY3RhbmdsZSBvZiB0aGUgZ2l2ZW4gc2V0IG9mIG5vZGVzIHdpdGhcclxuKiBvciB3aXRob3V0IHRoZSBtYXJnaW5zXHJcbiovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY3VsYXRlQm91bmRzID0gZnVuY3Rpb24gKGlzTWFyZ2luSW5jbHVkZWQsIG5vZGVzKVxyXG57XHJcbiAgICB2YXIgYm91bmRMZWZ0ID0gSW50ZWdlci5NQVhfVkFMVUU7XHJcbiAgICB2YXIgYm91bmRSaWdodCA9IEludGVnZXIuTUlOX1ZBTFVFO1xyXG4gICAgdmFyIGJvdW5kVG9wID0gSW50ZWdlci5NQVhfVkFMVUU7XHJcbiAgICB2YXIgYm91bmRCb3R0b20gPSBJbnRlZ2VyLk1JTl9WQUxVRTtcclxuICAgIHZhciBub2RlTGVmdDtcclxuICAgIHZhciBub2RlUmlnaHQ7XHJcbiAgICB2YXIgbm9kZVRvcDtcclxuICAgIHZhciBub2RlQm90dG9tO1xyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZDaGlsZHJlbiA9IG5vZGVzLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNoaWxkcmVuOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGxOb2RlID0gbm9kZXNbaV07XHJcbiAgICAgICAgbm9kZUxlZnQgPSBsTm9kZS5nZXRMZWZ0KCk7XHJcbiAgICAgICAgbm9kZVJpZ2h0ID0gbE5vZGUuZ2V0UmlnaHQoKTtcclxuICAgICAgICBub2RlVG9wID0gbE5vZGUuZ2V0VG9wKCk7XHJcbiAgICAgICAgbm9kZUJvdHRvbSA9IGxOb2RlLmdldEJvdHRvbSgpO1xyXG5cclxuICAgICAgICBpZiAoYm91bmRMZWZ0ID4gbm9kZUxlZnQpXHJcbiAgICAgICAgICAgIGJvdW5kTGVmdCA9IG5vZGVMZWZ0O1xyXG5cclxuICAgICAgICBpZiAoYm91bmRSaWdodCA8IG5vZGVSaWdodClcclxuICAgICAgICAgICAgYm91bmRSaWdodCA9IG5vZGVSaWdodDtcclxuXHJcbiAgICAgICAgaWYgKGJvdW5kVG9wID4gbm9kZVRvcClcclxuICAgICAgICAgICAgYm91bmRUb3AgPSBub2RlVG9wO1xyXG5cclxuICAgICAgICBpZiAoYm91bmRCb3R0b20gPCBub2RlQm90dG9tKVxyXG4gICAgICAgICAgICBib3VuZEJvdHRvbSA9IG5vZGVCb3R0b207XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKGlzTWFyZ2luSW5jbHVkZWQpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIG5ldyBSZWN0YW5nbGVEKGJvdW5kTGVmdCAtIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4sIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICBib3VuZFRvcCAtIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4sIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICBib3VuZFJpZ2h0IC0gYm91bmRMZWZ0ICsgMiAqIFNiZ25QRENvbnN0YW50cy5DT01QTEVYX01FTV9NQVJHSU4sXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIGJvdW5kQm90dG9tIC0gYm91bmRUb3AgKyAyICogU2JnblBEQ29uc3RhbnRzLkNPTVBMRVhfTUVNX01BUkdJTik7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIG5ldyBSZWN0YW5nbGVEKGJvdW5kTGVmdCwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIGJvdW5kVG9wLCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgYm91bmRSaWdodCAtIGJvdW5kTGVmdCwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIGJvdW5kQm90dG9tIC0gYm91bmRUb3ApO1xyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogY2FsY3VsYXRlcyB1c2VkQXJlYS90b3RhbEFyZWEgaW5zaWRlIHRoZSBjb21wbGV4ZXMgYW5kIHByaW50cyB0aGVtIG91dC5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jYWxjdWxhdGVGdWxsbmVzc09mQ29tcGxleGVzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGxhcmdlc3RDb21wbGV4ID0gbnVsbDtcclxuICAgIHZhciB0b3RhbEFyZWEgPSAwLjA7XHJcbiAgICB2YXIgdXNlZEFyZWEgPSAwLjA7XHJcbiAgICB2YXIgbWF4QXJlYSA9IE51bWJlci5NSU5fVkFMVUU7XHJcblxyXG4gICAgLy8gZmluZCB0aGUgbGFyZ2VzdCBjb21wbGV4IC0+IGFyZWFcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy5nZXRBbGxOb2RlcygpLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzID0gdGhpcy5nZXRBbGxOb2RlcygpW2ldO1xyXG4gICAgICAgIGlmICgocy50eXBlID09PSBTYmduUERDb25zdGFudHMuQ09NUExFWCkgJiYgXHJcbiAgICAgICAgICAgICgocy5nZXRXaWR0aCgpICogcy5nZXRIZWlnaHQoKSkgPiBtYXhBcmVhKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG1heEFyZWEgPSBzLmdldFdpZHRoKCkgKiBzLmdldEhlaWdodCgpO1xyXG4gICAgICAgICAgICBsYXJnZXN0Q29tcGxleCA9IHM7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHVzZWRBcmVhID0gdGhpcy5jYWxjdWxhdGVVc2VkQXJlYShsYXJnZXN0Q29tcGxleCk7XHJcbiAgICB0b3RhbEFyZWEgPSBsYXJnZXN0Q29tcGxleC5nZXRXaWR0aCgpICogbGFyZ2VzdENvbXBsZXguZ2V0SGVpZ2h0KCk7XHJcblxyXG4gICAgaWYgKHRoaXMuY29tcGFjdGlvbk1ldGhvZCA9PT0gU2JnblBETGF5b3V0LkRlZmF1bHRDb21wYWN0aW9uQWxnb3JpdGhtRW51bS5USUxJTkcpXHJcbiAgICAgICAgICAgIGNvbnNvbGUubG9nKFwiVGlsaW5nIHJlc3VsdHNcIik7XHJcbiAgICBlbHNlIGlmICh0aGlzLmNvbXBhY3Rpb25NZXRob2QgPT09IFNiZ25QRExheW91dC5EZWZhdWx0Q29tcGFjdGlvbkFsZ29yaXRobUVudW0uUE9MWU9NSU5PX1BBQ0tJTkcpXHJcbiAgICAgICAgICAgIGNvbnNvbGUubG9nKFwiUG9seW9taW5vIFBhY2tpbmcgcmVzdWx0c1wiKTtcclxuXHJcbiAgICBjb25zb2xlLmxvZyhcIiA9IFwiICsgdXNlZEFyZWEgLyB0b3RhbEFyZWEpO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY2FsY3VsYXRlcyB0aGUgdXNlZCBhcmVhIG9mIGEgZ2l2ZW4gY29tcGxleCBub2RlJ3MgY2hpbGRyZW5cclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5jYWxjdWxhdGVVc2VkQXJlYSA9IGZ1bmN0aW9uIChwYXJlbnQpXHJcbntcclxuICAgIHZhciB0b3RhbEFyZWEgPSAwO1xyXG4gICAgaWYgKHBhcmVudC5nZXRDaGlsZCgpID09IG51bGwpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIDAuMDtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHBhcmVudC5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIG5vZGUgPSBwYXJlbnQuZ2V0Q2hpbGQoKS5nZXROb2RlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAobm9kZS50eXBlICE9PSBTYmduUERDb25zdGFudHMuQ09NUExFWClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRvdGFsQXJlYSArPSBub2RlLmdldFdpZHRoKCkgKiBub2RlLmdldEhlaWdodCgpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0b3RhbEFyZWEgKz0gdGhpcy5jYWxjdWxhdGVVc2VkQXJlYShub2RlKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBcclxuICAgIHJldHVybiB0b3RhbEFyZWE7XHJcbn07XHJcblxyXG4vLyAqKioqKioqKioqKioqKioqKioqKiogU0VDVElPTiA6IE9WRVJSSURFTiBNRVRIT0RTICoqKioqKioqKioqKioqKioqKioqKlxyXG5cclxuLyoqXHJcbiAqIFRoaXMgbWV0aG9kIGNyZWF0ZXMgYSBuZXcgbm9kZSBhc3NvY2lhdGVkIHdpdGggdGhlIGlucHV0IHZpZXcgbm9kZS5cclxuICovXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUubmV3Tm9kZSA9IGZ1bmN0aW9uICh2Tm9kZSlcclxue1xyXG4gICAgcmV0dXJuIG5ldyBTYmduUEROb2RlKHRoaXMuZ3JhcGhNYW5hZ2VyLCBudWxsLCBudWxsLCB2Tm9kZSwgbnVsbCk7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgY3JlYXRlcyBhIG5ldyBlZGdlIGFzc29jaWF0ZWQgd2l0aCB0aGUgaW5wdXQgdmlldyBlZGdlLlxyXG4gKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5uZXdFZGdlID0gZnVuY3Rpb24gKHZFZGdlKVxyXG57XHJcbiAgICByZXR1cm4gbmV3IFNiZ25QREVkZ2UobnVsbCwgbnVsbCwgdkVkZ2UpO1xyXG59O1xyXG5cclxuLyoqXHJcbiAqIFRoaXMgbWV0aG9kIHBlcmZvcm1zIGxheW91dCBvbiBjb25zdHJ1Y3RlZCBsLWxldmVsIGdyYXBoLiBJdCByZXR1cm5zIHRydWVcclxuICogb24gc3VjY2VzcywgZmFsc2Ugb3RoZXJ3aXNlLlxyXG4gKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5sYXlvdXQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgYiA9IGZhbHNlO1xyXG5cclxuICAgIHRoaXMuZ3JvdXBaZXJvRGVncmVlTWVtYmVycygpO1xyXG4gICAgdGhpcy5hcHBseURGU09uQ29tcGxleGVzKCk7XHJcbiAgICBiID0gQ29TRUxheW91dC5wcm90b3R5cGUubGF5b3V0LmNhbGwodGhpcywgYXJndW1lbnRzKTtcclxuICAgIHRoaXMucmVwb3B1bGF0ZUNvbXBsZXhlcygpO1xyXG5cclxuICAgIHRoaXMuZ2V0QWxsTm9kZXMoKTtcclxuICAgIHJldHVybiBiO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgdXNlcyBjbGFzc2ljIGxheW91dCBtZXRob2QgKHdpdGhvdXQgbXVsdGktc2NhbGluZylcclxuKiBNb2RpZmljYXRpb246IGNyZWF0ZSBwb3J0IG5vZGVzIGFmdGVyIHJhbmRvbSBwb3NpdGlvbmluZ1xyXG4qL1xyXG4vL0BPdmVycmlkZVxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmNsYXNzaWNMYXlvdXQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB0aGlzLmNhbGN1bGF0ZU5vZGVzVG9BcHBseUdyYXZpdGF0aW9uVG8oKTtcclxuXHJcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5jYWxjTG93ZXN0Q29tbW9uQW5jZXN0b3JzKCk7XHJcbiAgICB0aGlzLmdyYXBoTWFuYWdlci5jYWxjSW5jbHVzaW9uVHJlZURlcHRocygpO1xyXG5cclxuICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLmdldFJvb3QoKS5jYWxjRXN0aW1hdGVkU2l6ZSgpO1xyXG4gICAgdGhpcy5jYWxjSWRlYWxFZGdlTGVuZ3RocygpO1xyXG5cclxuICAgIGlmICghdGhpcy5pbmNyZW1lbnRhbClcclxuICAgIHtcclxuICAgICAgICB2YXIgZm9yZXN0ID0gdGhpcy5nZXRGbGF0Rm9yZXN0KCk7XHJcblxyXG4gICAgICAgIGlmIChmb3Jlc3QubGVuZ3RoID4gMClcclxuICAgICAgICAvLyBUaGUgZ3JhcGggYXNzb2NpYXRlZCB3aXRoIHRoaXMgbGF5b3V0IGlzIGZsYXQgYW5kIGEgZm9yZXN0XHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLnBvc2l0aW9uTm9kZXNSYWRpYWxseShmb3Jlc3QpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAgLy8gVGhlIGdyYXBoIGFzc29jaWF0ZWQgd2l0aCB0aGlzIGxheW91dCBpcyBub3QgZmxhdCBvciBhIGZvcmVzdFxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGhpcy5wb3NpdGlvbk5vZGVzUmFuZG9tbHkoKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKCF0aGlzLmFyZVBvcnROb2Rlc0NyZWF0ZWQoKSlcclxuICAgIHtcclxuICAgICAgICB0aGlzLmNyZWF0ZVBvcnROb2RlcygpO1xyXG4gICAgICAgIHRoaXMuZ3JhcGhNYW5hZ2VyLnJlc2V0QWxsTm9kZXMoKTtcclxuICAgICAgICB0aGlzLmdyYXBoTWFuYWdlci5yZXNldEFsbE5vZGVzVG9BcHBseUdyYXZpdGF0aW9uKCk7XHJcbiAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIucmVzZXRBbGxFZGdlcygpO1xyXG4gICAgICAgIHRoaXMuY2FsY3VsYXRlTm9kZXNUb0FwcGx5R3Jhdml0YXRpb25UbygpO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB0aGlzLmluaXRTcHJpbmdFbWJlZGRlcigpO1xyXG4gICAgdGhpcy5ydW5TcHJpbmdFbWJlZGRlcigpO1xyXG5cclxuICAgIHJldHVybiB0cnVlO1xyXG59O1xyXG5cclxuXHJcbi8qKlxyXG4gKiBUaGlzIG1ldGhvZCBjYWxjdWxhdGVzIHRoZSBzcHJpbmcgZm9yY2VzIGZvciB0aGUgZW5kcyBvZiBlYWNoIG5vZGUuXHJcbiAqIE1vZGlmaWNhdGlvbjogZG8gbm90IGNhbGN1bGF0ZSBzcHJpbmcgZm9yY2UgZm9yIHJpZ2lkIGVkZ2VzXHJcbiAqL1xyXG4vL0BPdmVycmlkZVxyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLmNhbGNTcHJpbmdGb3JjZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgbEVkZ2VzID0gdGhpcy5nZXRBbGxFZGdlcygpO1xyXG4gICAgdmFyIGVkZ2U7XHJcblxyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsRWRnZXMubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgZWRnZSA9IGxFZGdlc1tpXTtcclxuXHJcbiAgICAgICAgaWYgKGVkZ2UudHlwZSAhPT0gU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0UpXHJcbiAgICAgICAgeyAgICBcclxuICAgICAgICAgICAgdGhpcy5jYWxjU3ByaW5nRm9yY2UoZWRnZSwgZWRnZS5pZGVhbExlbmd0aCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHQgXHJcbiAqIFRoaXMgbWV0aG9kIGNhbGN1bGF0ZXMgdGhlIHJlcHVsc2lvbiBmb3JjZXMgZm9yIGVhY2ggcGFpciBvZiBub2Rlcy5cclxuICogTW9kaWZpY2F0aW9uOiBEbyBub3QgY2FsY3VsYXRlIHJlcHVsc2lvbiBmb3IgcG9ydCAmIHByb2Nlc3Mgbm9kZXNcclxuICovXHJcbi8vQE92ZXJyaWRlXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY1JlcHVsc2lvbkZvcmNlcyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBpLCBqO1xyXG4gICAgdmFyIG5vZGVBLCBub2RlQjtcclxuICAgIHZhciBsTm9kZXMgPSB0aGlzLmdldEFsbE5vZGVzKCk7XHJcbiAgICB2YXIgcHJvY2Vzc2VkTm9kZVNldDtcclxuICAgIFxyXG4gICAgaWYgKHRoaXMudXNlRlJHcmlkVmFyaWFudClcclxuICAgIHtcclxuICAgICAgICAvLyBncmlkIGlzIGEgdmVjdG9yIG1hdHJpeCB0aGF0IGhvbGRzIENvU0VOb2Rlcy5cclxuICAgICAgICAvLyBiZSBzdXJlIHRvIGNvbnZlcnQgdGhlIE9iamVjdCB0eXBlIHRvIENvU0VOb2RlLlxyXG4gICAgICAgIGlmICh0aGlzLnRvdGFsSXRlcmF0aW9uc1xyXG4gICAgICAgICAgICAgICAgJSBTYmduUERDb25zdGFudHMuR1JJRF9DQUxDVUxBVElPTl9DSEVDS19QRVJJT0QgPT0gMSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuZ3JpZCA9IHRoaXMuY2FsY0dyaWQodGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpKTtcclxuICAgICAgICAgICAgXHJcbiAgICAgICAgICAgIC8vIHB1dCBhbGwgbm9kZXMgdG8gcHJvcGVyIGdyaWQgY2VsbHNcclxuICAgICAgICAgICAgZm9yIChpID0gMDsgaSA8IGxOb2Rlcy5sZW5ndGg7IGkrKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgbm9kZUEgPSBsTm9kZXNbaV07XHJcbiAgICAgICAgICAgICAgICB0aGlzLmFkZE5vZGVUb0dyaWQobm9kZUEsIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuZ3JpZCwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldExlZnQoKSwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy5ncmFwaE1hbmFnZXIuZ2V0Um9vdCgpLmdldFRvcCgpKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgcHJvY2Vzc2VkTm9kZVNldCA9IG5ldyBIYXNoU2V0KCk7XHJcblxyXG4gICAgICAgIC8vIGNhbGN1bGF0ZSByZXB1bHNpb24gZm9yY2VzIGJldHdlZW4gZWFjaCBub2RlcyBhbmQgaXRzIHN1cnJvdW5kaW5nXHJcbiAgICAgICAgZm9yIChpID0gMDsgaSA8IGxOb2Rlcy5sZW5ndGg7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG5vZGVBID0gbE5vZGVzW2ldO1xyXG4gICAgICAgICAgICB0aGlzLmNhbGN1bGF0ZVJlcHVsc2lvbkZvcmNlT2ZBTm9kZSh0aGlzLmdyaWQsIG5vZGVBLCBwcm9jZXNzZWROb2RlU2V0KTtcclxuICAgICAgICAgICAgcHJvY2Vzc2VkTm9kZVNldC5hZGQobm9kZUEpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICBmb3IgKGkgPSAwOyBpIDwgbE5vZGVzLmxlbmd0aDsgaSsrKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgbm9kZUEgPSBsTm9kZXNbaV07XHJcblxyXG4gICAgICAgICAgICBmb3IgKGogPSBpICsgMTsgaiA8IGxOb2Rlcy5sZW5ndGg7IGorKylcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgbm9kZUIgPSBsTm9kZXNbal07XHJcblxyXG4gICAgICAgICAgICAgICAgLy8gSWYgYm90aCBub2RlcyBhcmUgbm90IG1lbWJlcnMgb2YgdGhlIHNhbWUgZ3JhcGgsIHNraXAuXHJcbiAgICAgICAgICAgICAgICBpZiAobm9kZUEuZ2V0T3duZXIoKSAhPT0gbm9kZUIuZ2V0T3duZXIoKSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgICAgICBpZiAobm9kZUEudHlwZSAhPT0gbnVsbCAmJiBcclxuICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlICE9PSBudWxsICYmIFxyXG4gICAgICAgICAgICAgICAgICAgIG5vZGVBLmdldE93bmVyKCkgPT09IG5vZGVCLmdldE93bmVyKCkgJiYgXHJcbiAgICAgICAgICAgICAgICAgICAgKG5vZGVBLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICBub2RlQS50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQgfHwgXHJcbiAgICAgICAgICAgICAgICAgICAgIG5vZGVCLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpKVxyXG4gICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgICAgIHRoaXMuY2FsY1JlcHVsc2lvbkZvcmNlKG5vZGVBLCBub2RlQik7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgZmluZHMgc3Vycm91bmRpbmcgbm9kZXMgb2Ygbm9kZUEgaW4gcmVwdWxzaW9uIHJhbmdlLlxyXG4gKiBBbmQgY2FsY3VsYXRlcyB0aGUgcmVwdWxzaW9uIGZvcmNlcyBiZXR3ZWVuIG5vZGVBIGFuZCBpdHMgc3Vycm91bmRpbmcuXHJcbiAqIER1cmluZyB0aGUgY2FsY3VsYXRpb24sIGlnbm9yZXMgdGhlIG5vZGVzIHRoYXQgaGF2ZSBhbHJlYWR5IGJlZW4gcHJvY2Vzc2VkLlxyXG4gKiBNb2RpZmljYXRpb246IERvIG5vdCBjYWxjdWxhdGUgcmVwdWxzaW9uIGZvciBwb3J0ICYgcHJvY2VzcyBub2Rlc1xyXG4gKi9cclxuLy8gQE92ZXJyaWRlXHJcblNiZ25QRExheW91dC5wcm90b3R5cGUuY2FsY3VsYXRlUmVwdWxzaW9uRm9yY2VPZkFOb2RlID0gZnVuY3Rpb24gKGdyaWQsIG5vZGVBLCBwcm9jZXNzZWROb2RlU2V0KVxyXG57XHJcbiAgICB2YXIgaSwgajtcclxuXHJcbiAgICBpZiAodGhpcy50b3RhbEl0ZXJhdGlvbnMgJSBGRExheW91dENvbnN0YW50cy5HUklEX0NBTENVTEFUSU9OX0NIRUNLX1BFUklPRCA9PSAxKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzdXJyb3VuZGluZyA9IG5ldyBIYXNoU2V0KCk7XHJcbiAgICAgICAgdmFyIG5vZGVCO1xyXG5cclxuICAgICAgICBmb3IgKGkgPSAobm9kZUEuc3RhcnRYIC0gMSk7IGkgPCAobm9kZUEuZmluaXNoWCArIDIpOyBpKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBmb3IgKGogPSAobm9kZUEuc3RhcnRZIC0gMSk7IGogPCAobm9kZUEuZmluaXNoWSArIDIpOyBqKyspXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIGlmICghKChpIDwgMCkgfHwgKGogPCAwKSB8fCAoaSA+PSBncmlkLmxlbmd0aCkgfHwgKGogPj0gZ3JpZFswXS5sZW5ndGgpKSlcclxuICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICB2YXIgbnVtT2ZOb2RlcyA9IGdyaWRbaV1bal0ubGVuZ3RoO1xyXG4gICAgICAgICAgICAgICAgICAgIGZvciAodmFyIGsgPSAwOyBrIDwgbnVtT2ZOb2RlczsgaysrKVxyXG4gICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgbm9kZUIgPSBncmlkW2ldW2pdW2tdO1xyXG5cclxuICAgICAgICAgICAgICAgICAgICAgICAgLy8gSWYgYm90aCBub2RlcyBhcmUgbm90IG1lbWJlcnMgb2YgdGhlIHNhbWUgZ3JhcGgsXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC8vIG9yIGJvdGggbm9kZXMgYXJlIHRoZSBzYW1lLCBza2lwLlxyXG4gICAgICAgICAgICAgICAgICAgICAgICBpZiAoKG5vZGVBLmdldE93bmVyKCkgIT09IG5vZGVCLmdldE93bmVyKCkpIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKG5vZGVBID09PSBub2RlQikpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgICAgICAgICAgICAgICAgICB9XHJcblxyXG4gICAgICAgICAgICAgICAgICAgICAgICBpZiAobm9kZUEudHlwZSAhPT0gbnVsbCAmJiBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIG5vZGVCLnR5cGUgIT09IG51bGwgJiYgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlQS5nZXRPd25lcigpID09PSBub2RlQi5nZXRPd25lcigpICYmIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKG5vZGVBLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUIHx8IFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgIG5vZGVBLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVCB8fCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlID09PSBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVCB8fCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICBub2RlQi50eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgICAgICAgICAgICAgLy8gY2hlY2sgaWYgdGhlIHJlcHVsc2lvbiBmb3JjZSBiZXR3ZWVuXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC8vIG5vZGVBIGFuZCBub2RlQiBoYXMgYWxyZWFkeSBiZWVuIGNhbGN1bGF0ZWRcclxuICAgICAgICAgICAgICAgICAgICAgICAgaWYgKCFwcm9jZXNzZWROb2RlU2V0LmNvbnRhaW5zKG5vZGVCKSAmJiAhc3Vycm91bmRpbmcuY29udGFpbnMobm9kZUIpKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICB2YXIgZGlzdGFuY2VYID0gTWF0aC5hYnMobm9kZUEuZ2V0Q2VudGVyWCgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgLSBub2RlQi5nZXRDZW50ZXJYKCkpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgLSAoKG5vZGVBLmdldFdpZHRoKCkgLyAyKSArIChub2RlQlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAuZ2V0V2lkdGgoKSAvIDIpKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIHZhciBkaXN0YW5jZVkgPSBNYXRoLmFicyhub2RlQS5nZXRDZW50ZXJZKClcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAtIG5vZGVCLmdldENlbnRlclkoKSlcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAtICgobm9kZUEuZ2V0SGVpZ2h0KCkgLyAyKSArIChub2RlQlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAuZ2V0SGVpZ2h0KCkgLyAyKSk7XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIC8vIGlmIHRoZSBkaXN0YW5jZSBiZXR3ZWVuIG5vZGVBIGFuZCBub2RlQlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgLy8gaXMgbGVzcyB0aGVuIGNhbGN1bGF0aW9uIHJhbmdlXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICBpZiAoKGRpc3RhbmNlWCA8PSB0aGlzLnJlcHVsc2lvblJhbmdlKSAmJiBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAoZGlzdGFuY2VZIDw9IHRoaXMucmVwdWxzaW9uUmFuZ2UpKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIC8vIHRoZW4gYWRkIG5vZGVCIHRvIHN1cnJvdW5kaW5nIG9mIG5vZGVBXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgc3Vycm91bmRpbmcuYWRkKG5vZGVCKTtcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICBcclxuICAgICAgICBub2RlQS5zdXJyb3VuZGluZyA9IHN1cnJvdW5kaW5nLnNldDtcclxuICAgIH1cclxuXHJcbiAgICBmb3IgKGkgPSAwOyBpIDwgbm9kZUEuc3Vycm91bmRpbmcubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5jYWxjUmVwdWxzaW9uRm9yY2Uobm9kZUEsIG5vZGVBLnN1cnJvdW5kaW5nW2ldKTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIGNyZWF0ZXMgYSBwb3J0IG5vZGUgd2l0aCB0aGUgYXNzb2NpYXRlZCB0eXBlIChpbnB1dC9vdXRwdXRcclxuKiBwb3J0KVxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLm5ld1BvcnROb2RlID0gZnVuY3Rpb24gKHZOb2RlLCB0eXBlKVxyXG57XHJcbiAgICB2YXIgbiA9IG5ldyBTYmduUEROb2RlKHRoaXMuZ3JhcGhNYW5hZ2VyLCBudWxsLCBudWxsLCB2Tm9kZSwgbnVsbCk7XHJcbiAgICBuLnR5cGUgPSB0eXBlO1xyXG4gICAgbi5zZXRXaWR0aChTYmduUERDb25zdGFudHMuUE9SVF9OT0RFX0RFRkFVTFRfV0lEVEgpO1xyXG4gICAgbi5zZXRIZWlnaHQoU2JnblBEQ29uc3RhbnRzLlBPUlRfTk9ERV9ERUZBVUxUX0hFSUdIVCk7XHJcblxyXG4gICAgcmV0dXJuIG47XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBjcmVhdGVzIGFuIFNCR05Qcm9jZXNzTm9kZSBvYmplY3RcclxuKi9cclxuU2JnblBETGF5b3V0LnByb3RvdHlwZS5uZXdQcm9jZXNzTm9kZSA9IGZ1bmN0aW9uICh2Tm9kZSlcclxue1xyXG4gICAgcmV0dXJuIG5ldyBTYmduUHJvY2Vzc05vZGUodGhpcy5ncmFwaE1hbmFnZXIsIG51bGwsIG51bGwsIHZOb2RlKTtcclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIGNyZWF0ZXMgYSByaWdpZCBlZGdlLlxyXG4qL1xyXG5TYmduUERMYXlvdXQucHJvdG90eXBlLm5ld1JpZ2lkRWRnZSA9IGZ1bmN0aW9uICh2RWRnZSlcclxue1xyXG4gICAgdmFyIGUgPSBuZXcgU2JnblBERWRnZShudWxsLCBudWxsLCB2RWRnZSk7XHJcbiAgICBlLnR5cGUgPSBTYmduUERDb25zdGFudHMuUklHSURfRURHRTtcclxuICAgIHJldHVybiBlO1xyXG59O1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBTYmduUERMYXlvdXQ7XHJcbiIsInZhciBDb1NFTm9kZSA9IHJlcXVpcmUoJy4vQ29TRU5vZGUnKTtcclxudmFyIFNiZ25QRENvbnN0YW50cyA9IHJlcXVpcmUoJy4vU2JnblBEQ29uc3RhbnRzJyk7XHJcbnZhciBQb2ludEQgPSByZXF1aXJlKCcuL1BvaW50RCcpO1xyXG5cclxuLy8gVE9ETzogVGhlcmUgaXMgYW5vdGhlciBjb250cnVjdG9yIGF2YWlsYWJsZSBpbiBqYXZhLCBkbyB3ZSBuZWVkIGl0P1xyXG5cclxuZnVuY3Rpb24gU2JnblBETm9kZShnbSwgbG9jLCBzaXplLCB2Tm9kZSwgdHlwZSkgXHJcbntcclxuICAgIENvU0VOb2RlLmNhbGwodGhpcywgZ20sIGxvYywgc2l6ZSwgdk5vZGUpO1xyXG4gICAgXHJcbiAgICB0aGlzLnR5cGUgPSB0eXBlO1xyXG4gICAgdGhpcy52aXNpdGVkID0gZmFsc2U7XHJcbiAgICBcclxuICAgIHRoaXMubGFiZWwgPSB2Tm9kZSA/IHZOb2RlLmxhYmVsIDogbnVsbDtcclxuICAgIHRoaXMuaXNEdW1teUNvbXBvdW5kID0gZmFsc2U7XHJcbn1cclxuXHJcblNiZ25QRE5vZGUucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShDb1NFTm9kZS5wcm90b3R5cGUpO1xyXG5mb3IgKHZhciBwcm9wIGluIENvU0VOb2RlKSB7XHJcbiAgU2JnblBETm9kZVtwcm9wXSA9IENvU0VOb2RlW3Byb3BdO1xyXG59XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5jb3B5ID0gZnVuY3Rpb24gKC8qU2JnblBETm9kZSovIG5vZGUpIFxyXG57XHJcbiAgICB0aGlzLnR5cGUgPSBub2RlLnR5cGU7XHJcbiAgICB0aGlzLmxhYmVsID0gbm9kZS5sYWJlbDtcclxuICAgIHRoaXMuc2V0Q2VudGVyKG5vZGUuZ2V0Q2VudGVyWCgpLCBub2RlLmdldENlbnRlclkoKSk7XHJcbiAgICB0aGlzLnNldENoaWxkKG5vZGUuZ2V0Q2hpbGQoKSk7XHJcbiAgICB0aGlzLnNldEhlaWdodChub2RlLmdldEhlaWdodCgpKTtcclxuICAgIHRoaXMuc2V0TG9jYXRpb24obm9kZS5nZXRMb2NhdGlvbigpLngsIG5vZGUuZ2V0TG9jYXRpb24oKS55KTtcclxuICAgIHRoaXMuc2V0TmV4dChub2RlLmdldE5leHQoKSk7XHJcbiAgICB0aGlzLnNldE93bmVyKG5vZGUuZ2V0T3duZXIoKSk7XHJcbiAgICB0aGlzLnNldFByZWQxKG5vZGUuZ2V0UHJlZDEoKSk7XHJcbiAgICB0aGlzLnNldFByZWQyKG5vZGUuZ2V0UHJlZDIoKSk7XHJcbiAgICB0aGlzLnNldFdpZHRoKG5vZGUuZ2V0V2lkdGgoKSk7XHJcbn07XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5nZXRTcHJpbmdGb3JjZVggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy5zcHJpbmdGb3JjZVg7XHJcbn07XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5pc0NvbXBsZXggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICBpZiAodGhpcy50eXBlICE9IHVuZGVmaW5lZClcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLkNPTVBMRVgpID09PSAwO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiAtMTtcclxuICAgIH1cclxuICAgIFxyXG59O1xyXG5cclxuU2JnblBETm9kZS5wcm90b3R5cGUuaXNJbnB1dFBvcnQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gdGhpcy50eXBlLmxvY2FsZUNvbXBhcmUoU2JnblBEQ29uc3RhbnRzLklOUFVUX1BPUlQpID09PSAwO1xyXG59O1xyXG5cclxuU2JnblBETm9kZS5wcm90b3R5cGUuaXNPdXRwdXRQb3J0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMudHlwZS5sb2NhbGVDb21wYXJlKFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVCkgPT09IDA7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgY2hlY2tzIGlmIHRoZSBnaXZlbiBub2RlIGNvbnRhaW5zIGFueSB1bm1hcmtlZCBjb21wbGV4IG5vZGVzXHJcbiAqIGluIGl0cyBjaGlsZCBncmFwaC5cclxuKi9cclxuU2JnblBETm9kZS5wcm90b3R5cGUuY29udGFpbnNVbm1hcmtlZENvbXBsZXggPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICBpZiAodGhpcy5nZXRDaGlsZCgpID09IG51bGwpXHJcbiAgICB7XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIHZhciBudW1PZkNoaWxkcmVuID0gdGhpcy5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoO1xyXG4gICAgICAgIGZvciAodmFyIGluZGV4PTA7IGluZGV4PG51bU9mQ2hpbGRyZW47IGluZGV4KyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgY2hpbGQgPSB0aGlzLmdldENoaWxkKCkuZ2V0Tm9kZXMoKVtpbmRleF07XHJcbiAgICAgICAgICAgIGlmIChjaGlsZC5pc0NvbXBsZXgoKSAmJiAhY2hpbGQudmlzaXRlZClcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblBETm9kZS5wcm90b3R5cGUucmVzZXRGb3JjZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB0aGlzLnNwcmluZ0ZvcmNlWCA9IDA7XHJcbiAgICB0aGlzLnNwcmluZ0ZvcmNlWSA9IDA7XHJcbiAgICB0aGlzLnJlcHVsc2lvbkZvcmNlWCA9IDA7XHJcbiAgICB0aGlzLnJlcHVsc2lvbkZvcmNlWSA9IDA7XHJcbn07XHJcblxyXG5TYmduUEROb2RlLnByb3RvdHlwZS5yb3RhdGVOb2RlID0gZnVuY3Rpb24gKC8qUG9pbnREKi8gb3JpZ2luLCAvKmludCovIHJvdGF0aW9uRGVncmVlKVxyXG57XHJcbiAgICB2YXIgcmVsYXRpdmVQdCA9IG5ldyBQb2ludEQoXHJcbiAgICAgICAgICAgICh0aGlzLmdldENlbnRlclgoKSAtIG9yaWdpbi54KSwgKHRoaXMuZ2V0Q2VudGVyWSgpIC0gb3JpZ2luLnkpKTtcclxuICAgIHZhciByb3RhdGVkUHQgPSBuZXcgUG9pbnREKFxyXG4gICAgICAgICAgICAoLU1hdGguc2lnbnVtKHJvdGF0aW9uRGVncmVlKSAqIHJlbGF0aXZlUHQueSksIFxyXG4gICAgICAgICAgICAoTWF0aC5zaWdudW0ocm90YXRpb25EZWdyZWUpICogcmVsYXRpdmVQdC54KSk7XHJcblxyXG4gICAgdGhpcy5zZXRDZW50ZXIocm90YXRlZFB0LnggKyBvcmlnaW4ueCwgcm90YXRlZFB0LnkgKyBvcmlnaW4ueSk7XHJcblxyXG4gICAgdmFyIG5ld0hlaWdodCA9IHRoaXMuZ2V0V2lkdGgoKTtcclxuICAgIHZhciBuZXdXaWR0aCA9IHRoaXMuZ2V0SGVpZ2h0KCk7XHJcbiAgICB0aGlzLnNldFdpZHRoKG5ld1dpZHRoKTtcclxuICAgIHRoaXMuc2V0SGVpZ2h0KG5ld0hlaWdodCk7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgaXMgdXNlZCBmb3IgcG9ydCBub2RlcyBvbmx5XHJcbiAqL1xyXG5TYmduUEROb2RlLnByb3RvdHlwZS5jYWxjQXZlcmFnZVBvaW50ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIGF2ZXJhZ2VQbnQgPSBuZXcgUG9pbnREKDAuMCwgMC4wKTtcclxuICAgIFxyXG4gICAgdmFyIG51bU9mRWRnZXMgPSB0aGlzLmdldEVkZ2VzKCkubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaW5kZXg9MDsgaW5kZXg8bnVtT2ZFZGdlczsgaW5kZXgrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMuZ2V0RWRnZXMoKVtpbmRleF07XHJcbiAgICAgICAgXHJcbiAgICAgICAgaWYgKGVkZ2UudHlwZS5sb2NhbGVDb21wYXJlKFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFKSA9PT0gMClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGNvbnRpbnVlO1xyXG4gICAgICAgIH1cclxuICAgICAgICBcclxuICAgICAgICBhdmVyYWdlUG50LnggKz0gZWRnZS5nZXRPdGhlckVuZCh0aGlzKS5nZXRDZW50ZXJYKCk7XHJcbiAgICAgICAgYXZlcmFnZVBudC55ICs9IGVkZ2UuZ2V0T3RoZXJFbmQodGhpcykuZ2V0Q2VudGVyWSgpO1xyXG4gICAgfVxyXG5cclxuICAgIGF2ZXJhZ2VQbnQueCAvPSAodGhpcy5nZXRFZGdlcygpLmxlbmd0aCAtIDEpO1xyXG4gICAgYXZlcmFnZVBudC55IC89ICh0aGlzLmdldEVkZ2VzKCkubGVuZ3RoIC0gMSk7XHJcblxyXG4gICAgcmV0dXJuIGF2ZXJhZ2VQbnQ7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCByZXR1cm5zIHRoZSBuZWlnaGJvcnMgb2YgYSBnaXZlbiBub2RlLiBOb3RpY2UgdGhhdCB0aGUgZ3JhcGhcclxuKiBpcyBkaXJlY3RlZC4gVGhlcmVmb3JlIGVkZ2VzIHNob3VsZCBoYXZlIHRoZSBnaXZlbiBub2RlIGFzIHRoZSBzb3VyY2VcclxuKiBub2RlLlxyXG4qL1xyXG5TYmduUEROb2RlLnByb3RvdHlwZS5nZXRDaGlsZHJlbk5laWdoYm9ycyA9IGZ1bmN0aW9uICgvKlN0cmluZyovIGVkZ2VUeXBlKVxyXG57XHJcbiAgICB2YXIgbmVpZ2hib3JzID0gW107XHJcblxyXG4gICAgZm9yICh2YXIgaW5kZXggPSAwOyBpbmRleDx0aGlzLmdldEVkZ2VzKCkubGVuZ3RoOyBpbmRleCsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBlZGdlID0gdGhpcy5nZXRFZGdlcygpW2luZGV4XTtcclxuXHJcbiAgICAgICAgaWYgKChlZGdlLmdldFNvdXJjZSgpPT0gdGhpcykgJiYgKGVkZ2UuZ2V0VGFyZ2V0KCkgIT0gdGhpcykpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgbm9kZSA9IGVkZ2UuZ2V0VGFyZ2V0KCk7XHJcblxyXG4gICAgICAgICAgICBpZiAoZWRnZVR5cGUgIT0gbnVsbCAmJiBlZGdlLnR5cGUubG9jYWxlQ29tcGFyZShlZGdlVHlwZSkgPT09IDApXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIG5laWdoYm9ycy5wdXNoKG5vZGUpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGlmIChlZGdlVHlwZSA9PSBudWxsKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBuZWlnaGJvcnMucHVzaChub2RlKTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIHJldHVybiBuZWlnaGJvcnM7XHJcbn07XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgdXBkYXRlcyB0aGUgYm91bmRzIG9mIHRoaXMgY29tcG91bmQgbm9kZS4gSWYgdGhlIG5vZGUgaXMgYVxyXG4gKiBkdW1teSBjb21wb3VuZCwgZG8gbm90IGluY2x1ZGUgbGFiZWwgYW5kIGV4dHJhIG1hcmdpbnMuXHJcbiAqL1xyXG5TYmduUEROb2RlLnByb3RvdHlwZS51cGRhdGVCb3VuZHMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICAvLyBUT0RPOiBEbyB3ZSBuZWVkIGhhbmRsZSBhc3NlcnRpb25zP1xyXG4gICAgLyphc3NlcnQgdGhpcy5nZXRDaGlsZCgpICE9IG51bGw7Ki9cclxuXHJcbiAgICBpZiAodGhpcy5nZXRDaGlsZCgpLmdldE5vZGVzKCkubGVuZ3RoICE9PSAwKVxyXG4gICAge1xyXG4gICAgICAgIC8vIHdyYXAgdGhlIGNoaWxkcmVuIG5vZGVzIGJ5IHJlLWFycmFuZ2luZyB0aGUgYm91bmRhcmllc1xyXG4gICAgICAgIHZhciBjaGlsZEdyYXBoID0gdGhpcy5nZXRDaGlsZCgpO1xyXG4gICAgICAgIGNoaWxkR3JhcGgudXBkYXRlQm91bmRzKHRydWUpO1xyXG5cclxuICAgICAgICB0aGlzLnJlY3QueCA9IGNoaWxkR3JhcGguZ2V0TGVmdCgpO1xyXG4gICAgICAgIHRoaXMucmVjdC55ID0gY2hpbGRHcmFwaC5nZXRUb3AoKTtcclxuXHJcbiAgICAgICAgaWYgKCh0aGlzLnR5cGUgIT0gbnVsbCkgJiYgXHJcbiAgICAgICAgICAgICh0aGlzLnR5cGUubG9jYWxlQ29tcGFyZShTYmduUERDb25zdGFudHMuRFVNTVlfQ09NUE9VTkQpID09PSAwKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuc2V0V2lkdGgoY2hpbGRHcmFwaC5nZXRSaWdodCgpIC0gY2hpbGRHcmFwaC5nZXRMZWZ0KCkpO1xyXG4gICAgICAgICAgICB0aGlzLnNldEhlaWdodChjaGlsZEdyYXBoLmdldEJvdHRvbSgpIC0gY2hpbGRHcmFwaC5nZXRUb3AoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuc2V0V2lkdGgoY2hpbGRHcmFwaC5nZXRSaWdodCgpIC0gY2hpbGRHcmFwaC5nZXRMZWZ0KCkgKyAyXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAqIFNiZ25QRENvbnN0YW50cy5DT01QT1VORF9OT0RFX01BUkdJTik7XHJcbiAgICAgICAgICAgIHRoaXMuc2V0SGVpZ2h0KGNoaWxkR3JhcGguZ2V0Qm90dG9tKCkgLSBjaGlsZEdyYXBoLmdldFRvcCgpICsgMlxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgKiBTYmduUERDb25zdGFudHMuQ09NUE9VTkRfTk9ERV9NQVJHSU5cclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLkxBQkVMX0hFSUdIVCk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG4gICAgICAgIFxyXG5tb2R1bGUuZXhwb3J0cyA9IFNiZ25QRE5vZGU7XHJcbiIsInZhciBJR2VvbWV0cnkgPSByZXF1aXJlKCcuL0lHZW9tZXRyeScpO1xyXG52YXIgUG9pbnREID0gcmVxdWlyZSgnLi9Qb2ludEQnKTtcclxuXHJcbnZhciBTYmduUEROb2RlID0gcmVxdWlyZSgnLi9TYmduUEROb2RlJyk7XHJcbnZhciBTYmduUERFZGdlID0gcmVxdWlyZSgnLi9TYmduUERFZGdlJyk7XHJcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5PcmllbnRhdGlvbkVudW0gPSBcclxue1xyXG4gICAgQk9UVE9NX1RPX1RPUCA6IDAsIFxyXG4gICAgVE9QX1RPX0JPVFRPTSA6IDEsXHJcbiAgICBMRUZUX1RPX1JJR0hUIDogMixcclxuICAgIFJJR0hUX1RPX0xFRlQgOiAzXHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLlJvdGF0aW9uUHJpb3JpdHlFbnVtID0gXHJcbntcclxuICAgIE5JTkVUWV9ERUdSRUUgOiAwLCBcclxuICAgIFNXQVAgOiAxLFxyXG4gICAgTk9fUk9UQVRJT04gOiAyXHJcbn07XHJcblxyXG5mdW5jdGlvbiBTYmduUHJvY2Vzc05vZGUoZ20sIGxvYywgc2l6ZSwgdk5vZGUsIHR5cGUpIFxyXG57XHJcbiAgICBTYmduUEROb2RlLmNhbGwodGhpcywgZ20sIGxvYywgc2l6ZSwgdk5vZGUsIHR5cGUpO1xyXG5cclxuICAgIHRoaXMubmV0Um90YXRpb25hbEZvcmNlID0gMDtcclxuICAgIHRoaXMuY29uc3VtcHRpb25FZGdlcyA9IFtdO1xyXG4gICAgdGhpcy5wcm9kdWN0RWRnZXMgPSBbXTtcclxuICAgIHRoaXMuZWZmZWN0b3JFZGdlcyA9IFtdO1xyXG4gICAgXHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kID0gbnVsbDtcclxuICAgIHRoaXMuaW5wdXRQb3J0ID0gbnVsbDtcclxuICAgIHRoaXMub3V0cHV0UG9ydCA9IG51bGw7XHJcbiAgICBcclxuICAgIHRoaXMub3JpZW50YXRpb24gPSBudWxsO1xyXG4gICAgdGhpcy5yb3RhdGlvblByaW9yaXR5ID0gbnVsbDtcclxuICAgIFxyXG4gICAgdGhpcy5pZGVhbEVkZ2VMZW5ndGggPSAwLjA7XHJcbiAgICB0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSA9IDAuMDtcclxuICAgIHRoaXMucHJvcGVyRWRnZUNvdW50ID0gMC4wO1xyXG59XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShTYmduUEROb2RlLnByb3RvdHlwZSk7XHJcbmZvciAodmFyIHByb3AgaW4gU2JnblBETm9kZSkge1xyXG4gIFNiZ25Qcm9jZXNzTm9kZVtwcm9wXSA9IFNiZ25QRE5vZGVbcHJvcF07XHJcbn1cclxuXHJcbi8qKlxyXG4qIENvbm5lY3QgdGhlIHBvcnQgbm9kZSB0byBpdHMgcHJvY2VzcyBub2RlIChwYXJlbnQpIGFuZCBjb25uZWN0IHRoZSBlZGdlc1xyXG4qIG9mIG5laWdoYm9yIG5vZGVzIHRvIHRoZSBwb3J0IG5vZGUgYnkgY29uc2lkZXJpbmcgdGhlaXIgdHlwZXMgKGZvciBib3RoXHJcbiogaW5wdXQgcG9ydCBhbmQgb3V0cHV0IHBvcnQpXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUucmVjb25uZWN0RWRnZXMgPSBmdW5jdGlvbiAoaWRlYWxFZGdlTGVuZ3RoKVxyXG57XHJcbiAgICB0aGlzLmlkZWFsRWRnZUxlbmd0aCA9IGlkZWFsRWRnZUxlbmd0aDtcclxuICAgIFxyXG4gICAgLy8gY2hhbmdlIGNvbm5lY3Rpb25zIGZyb20gcHJvY2VzcyBub2RlJm5laWdoYm9ycyB0byBwb3J0Jm5laWdoYm9ycy5cclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgdGhpcy5nZXRFZGdlcygpLmxlbmd0aDsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBzRWRnZSA9IHRoaXMuZ2V0RWRnZXMoKVtpXTtcclxuXHJcbiAgICAgICAgaWYgKHNFZGdlLnR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5DT05TVU1QVElPTilcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuZ2V0RWRnZXMoKS5zcGxpY2UodGhpcy5nZXRFZGdlcygpLmluZGV4T2Yoc0VkZ2UpLCAxKTtcclxuXHJcbiAgICAgICAgICAgIHNFZGdlLnNldFRhcmdldCh0aGlzLmlucHV0UG9ydCk7XHJcbiAgICAgICAgICAgIHRoaXMuaW5wdXRQb3J0LmdldEVkZ2VzKCkucHVzaChzRWRnZSk7XHJcbiAgICAgICAgICAgIGktLTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAoc0VkZ2UudHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLlBST0RVQ1RJT04pXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLmdldEVkZ2VzKCkuc3BsaWNlKHRoaXMuZ2V0RWRnZXMoKS5pbmRleE9mKHNFZGdlKSwgMSk7XHJcblxyXG4gICAgICAgICAgICBzRWRnZS5zZXRTb3VyY2UodGhpcy5vdXRwdXRQb3J0KTtcclxuICAgICAgICAgICAgdGhpcy5vdXRwdXRQb3J0LmdldEVkZ2VzKCkucHVzaChzRWRnZSk7XHJcbiAgICAgICAgICAgIGktLTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAoc0VkZ2UuaXNFZmZlY3RvcigpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGhpcy5nZXRFZGdlcygpLnNwbGljZSh0aGlzLmdldEVkZ2VzKCkuaW5kZXhPZihzRWRnZSksIDEpO1xyXG5cclxuICAgICAgICAgICAgc0VkZ2Uuc2V0VGFyZ2V0KHRoaXMucGFyZW50Q29tcG91bmQpO1xyXG4gICAgICAgICAgICB0aGlzLnBhcmVudENvbXBvdW5kLmdldEVkZ2VzKCkucHVzaChzRWRnZSk7XHJcbiAgICAgICAgICAgIGktLTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNvbm5lY3ROb2RlcyA9IGZ1bmN0aW9uIChwYXJlbnRDb21wb3VuZCwgaW5wdXRQb3J0LCBvdXRwdXRQb3J0KVxyXG57XHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kID0gcGFyZW50Q29tcG91bmQ7XHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kLmlzRHVtbXlDb21wb3VuZCA9IHRydWU7XHJcbiAgICB0aGlzLmlucHV0UG9ydCA9IGlucHV0UG9ydDtcclxuICAgIHRoaXMub3V0cHV0UG9ydCA9IG91dHB1dFBvcnQ7XHJcbiAgICB0aGlzLm9yaWVudGF0aW9uID0gdGhpcy5PcmllbnRhdGlvbkVudW0uTEVGVF9UT19SSUdIVDtcclxuXHJcbiAgICAvLyBpbml0aWFsIHBsYWNlbWVudC4gcGxhY2UgaW5wdXQgdG8gdGhlIGxlZnQgb2YgdGhlIHByb2Nlc3Mgbm9kZSxcclxuICAgIC8vIG91dHB1dCB0byB0aGUgcmlnaHRcclxuICAgIG91dHB1dFBvcnQuc2V0Q2VudGVyKHRoaXMuZ2V0Q2VudGVyWCgpICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RILCBcclxuICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuZ2V0Q2VudGVyWSgpKTtcclxuICAgIGlucHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKCkgLSBTYmduUERDb25zdGFudHMuUklHSURfRURHRV9MRU5HVEgsIFxyXG4gICAgICAgICAgICAgICAgICAgICAgICB0aGlzLmdldENlbnRlclkoKSk7XHJcbn07XHJcblxyXG4vKipcclxuKiBDaGVjayBpZiB0aGUgcHJvY2VzcyBpcyBlbGlnaWJsZSBmb3Igcm90YXRpb24uIEZpcnN0IGNoZWNrIGlmIGFcclxuKiAxODAtZGVncmVlIGlzIHBvc3NpYmxlIChhcyBpdCBpcyBtb3JlIGNyaXRpY2FsKS5cclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5pc1JvdGF0aW9uTmVjZXNzYXJ5ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgLy8gbm9ybWFsaXplIHRoZSBhbW91bnQgKHBlciBpdGVyYXRpb24pXHJcbiAgICB0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSAvPSAoU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OQUxfRk9SQ0VfSVRFUkFUSU9OX0NPVU5UICogXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICh0aGlzLmNvbnN1bXB0aW9uRWRnZXMubGVuZ3RoICsgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLnByb2R1Y3RFZGdlcy5sZW5ndGggKyBcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuZWZmZWN0b3JFZGdlcy5sZW5ndGgpKTtcclxuXHJcbiAgICBpZiAodGhpcy5pc1N3YXBBdmFpbGFibGUoKSlcclxuICAgIHtcclxuICAgICAgICAgdGhpcy5yb3RhdGlvblByaW9yaXR5ID0gdGhpcy5Sb3RhdGlvblByaW9yaXR5RW51bS5TV0FQO1xyXG4gICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2UgaWYgKE1hdGguYWJzKHRoaXMubmV0Um90YXRpb25hbEZvcmNlKSA+IFNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICB7XHJcbiAgICAgICAgIHRoaXMucm90YXRpb25Qcmlvcml0eSA9IHRoaXMuUm90YXRpb25Qcmlvcml0eUVudW0uTklORVRZX0RFR1JFRTtcclxuICAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgIHRoaXMucm90YXRpb25Qcmlvcml0eSA9IHRoaXMuUm90YXRpb25Qcmlvcml0eUVudW0uTk9fUk9UQVRJT047XHJcbiAgICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIElmIHRoZSBwZXJjZW50YWdlIG9mIG9idHVzZSBhbmdsZXMgZXhjZWVkcyB0aGUgdGhyZXNob2xkLCBzd2FwIGlzXHJcbiogcmVxdWlyZWQuXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuaXNTd2FwQXZhaWxhYmxlID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdmFyIG9idHVzZUFuZ2xlQ250ID0gMC4wO1xyXG4gICAgdmFyIGFjdXRlQW5nbGVDbnQgPSAwLjA7XHJcblxyXG4gICAgdmFyIG51bU9mQ29uc3VtcHRpb25FZGdlcyA9IHRoaXMuY29uc3VtcHRpb25FZGdlcy5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZDb25zdW1wdGlvbkVkZ2VzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSB0aGlzLmNvbnN1bXB0aW9uRWRnZXNbaV07XHJcbiAgICAgICAgXHJcbiAgICAgICAgaWYgKE1hdGguYWJzKGVkZ2UuY29ycmVzcG9uZGluZ0FuZ2xlKSA+IDkwKVxyXG4gICAgICAgICAgICBvYnR1c2VBbmdsZUNudCsrO1xyXG4gICAgICAgIGVsc2VcclxuICAgICAgICAgICAgYWN1dGVBbmdsZUNudCsrO1xyXG4gICAgfVxyXG4gICAgXHJcbiAgICB2YXIgbnVtT2ZQcm9kdWN0RWRnZXMgPSB0aGlzLnByb2R1Y3RFZGdlcy5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZQcm9kdWN0RWRnZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMucHJvZHVjdEVkZ2VzW2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIGlmIChNYXRoLmFicyhlZGdlLmNvcnJlc3BvbmRpbmdBbmdsZSkgPiA5MClcclxuICAgICAgICAgICAgICAgIG9idHVzZUFuZ2xlQ250Kys7XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICAgICAgYWN1dGVBbmdsZUNudCsrO1xyXG4gICAgfVxyXG5cclxuICAgIGlmIChvYnR1c2VBbmdsZUNudCAvIChvYnR1c2VBbmdsZUNudCArIGFjdXRlQW5nbGVDbnQpID4gU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzE4MF9ERUdSRUUpXHJcbiAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICBlbHNlXHJcbiAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2Qgcm90YXRlcyB0aGUgYXNzb2NpYXRlZCBjb21wb3VuZCAoYW5kIGl0cyBjaGlsZHJlbjogcHJvY2Vzc1xyXG4qIGFuZCBwb3J0cykuXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuYXBwbHlSb3RhdGlvbiA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIGlmICh0aGlzLnJvdGF0aW9uUHJpb3JpdHkgPT09IHRoaXMuUm90YXRpb25Qcmlvcml0eUVudW0uTklORVRZX0RFR1JFRSlcclxuICAgIHtcclxuICAgICAgICBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSA+IFNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoOTApO1xyXG4gICAgICAgICAgICAgICAgdGhpcy5vcmllbnRhdGlvbiA9IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQ7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgZWxzZSBpZiAobmV0Um90YXRpb25hbEZvcmNlIDwgLVNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkJPVFRPTV9UT19UT1ApXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodGhpcy5uZXRSb3RhdGlvbmFsRm9yY2UgPCAtU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzkwX0RFR1JFRSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5yb3RhdGVDb21wb3VuZCg5MCk7XHJcbiAgICAgICAgICAgICAgICB0aGlzLm9yaWVudGF0aW9uID0gdGhpcy5PcmllbnRhdGlvbkVudW0uTEVGVF9UT19SSUdIVDtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICBlbHNlIGlmICh0aGlzLm5ldFJvdGF0aW9uYWxGb3JjZSA+IFNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodGhpcy5uZXRSb3RhdGlvbmFsRm9yY2UgPiBTYmduUERDb25zdGFudHMuUk9UQVRJT05fOTBfREVHUkVFKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICB0aGlzLnJvdGF0ZUNvbXBvdW5kKDkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHRoaXMubmV0Um90YXRpb25hbEZvcmNlIDwgLVNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSAgdGhpcy5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHRoaXMubmV0Um90YXRpb25hbEZvcmNlIDwgLVNiZ25QRENvbnN0YW50cy5ST1RBVElPTl85MF9ERUdSRUUpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMucm90YXRlQ29tcG91bmQoLTkwKTtcclxuICAgICAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHRoaXMubmV0Um90YXRpb25hbEZvcmNlID4gU2JnblBEQ29uc3RhbnRzLlJPVEFUSU9OXzkwX0RFR1JFRSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgdGhpcy5yb3RhdGVDb21wb3VuZCg5MCk7XHJcbiAgICAgICAgICAgICAgICB0aGlzLm9yaWVudGF0aW9uID0gIHRoaXMuT3JpZW50YXRpb25FbnVtLlRPUF9UT19CT1RUT007XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgZWxzZSBpZiAodGhpcy5yb3RhdGlvblByaW9yaXR5ID09PSB0aGlzLlJvdGF0aW9uUHJpb3JpdHlFbnVtLlNXQVApXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIHRlbXBDZW50ZXIgPSB0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXIoKTtcclxuICAgICAgICBcclxuICAgICAgICB0aGlzLmlucHV0UG9ydC5zZXRDZW50ZXIodGhpcy5vdXRwdXRQb3J0LmdldENlbnRlclgoKSwgXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWSgpKTtcclxuICAgICAgICB0aGlzLm91dHB1dFBvcnQuc2V0Q2VudGVyKHRlbXBDZW50ZXIueCwgdGVtcENlbnRlci55KTtcclxuXHJcbiAgICAgICAgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlRPUF9UT19CT1RUT00pXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QO1xyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkJPVFRPTV9UT19UT1ApXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NO1xyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQpXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUO1xyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICAgICAgICAgIHRoaXMub3JpZW50YXRpb24gPSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMubmV0Um90YXRpb25hbEZvcmNlID0gMDtcclxufTtcclxuXHJcblxyXG4vKipcclxuKiBHaXZlbiBhIGNvbXBvdW5kIG5vZGUsIHRoaXMgbWV0aG9kIHJlY3Vyc2l2ZWx5IHJvdGF0ZXMgdGhlIGNvbXBvdW5kIG5vZGVcclxuKiBhbmQgaXRzIG1lbWJlcnMuXHJcbiovXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUucm90YXRlQ29tcG91bmQgPSBmdW5jdGlvbiAocm90YXRpb25EZWdyZWUpXHJcbntcclxuICAgIHRoaXMucm90YXRlTm9kZSh0aGlzLmdldENlbnRlcigpLCByb3RhdGlvbkRlZ3JlZSk7XHJcbiAgICB0aGlzLmlucHV0UG9ydC5yb3RhdGVOb2RlKHRoaXMuZ2V0Q2VudGVyKCksIHJvdGF0aW9uRGVncmVlKTtcclxuICAgIHRoaXMub3V0cHV0UG9ydC5yb3RhdGVOb2RlKHRoaXMuZ2V0Q2VudGVyKCksIHJvdGF0aW9uRGVncmVlKTtcclxuICAgIHRoaXMucGFyZW50Q29tcG91bmQudXBkYXRlQm91bmRzKCk7XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNhbGNSb3RhdGlvbmFsRm9yY2VzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdGhpcy5uZXRSb3RhdGlvbmFsRm9yY2UgKz0gdGhpcy5jYWxjUHJvcGVybHlPcmllbnRlZEVkZ2VzKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCBjYWxjdWxhdGVzIGFsbCBhbmdsZXMgYmV0d2VlbiBwcm9jZXNzIGFuZCBpdHMgZWRnZXMgKHByb2QsXHJcbiogY29ucywgZWZmKSBhbmQgbWFya3MgdGhlbSBhcyBwcm9wZXJseSBvcmllbnRlZCBvciBub3QuIFJldHVybmVkIHZhbHVlIGlzXHJcbiogdGhlIGFtb3VudCBvZiBkZXNpcmUgdG8gcm90YXRlIGF0IHRoaXMgc3RlcC4gVGhlIHJldHVybmVkIHZhbHVlIHNob3VsZCBiZVxyXG4qIHRoZW4gbWFudWFsbHkgYWRkZWQgdG8gbmV0Um90YXRpb25hbEZvcmNlIChpZiBhaW0gaXMgdG8gY2FsY3VsYXRlXHJcbiogbmV0cm90YXRpb25hbGZvcmNlKVxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNhbGNQcm9wZXJseU9yaWVudGVkRWRnZXMgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICB2YXIgaW5wdXRSb3RTdW0gPSAwO1xyXG4gICAgdmFyIG91dHB1dFJvdFN1bSA9IDA7XHJcbiAgICB2YXIgZWZmZWN0b3JSb3RTdW0gPSAwO1xyXG4gICAgdmFyIHN0ZXBTdW0gPSAwO1xyXG4gICAgdmFyIHJlc3VsdDtcclxuICAgIHRoaXMucHJvcGVyRWRnZUNvdW50ID0gMDtcclxuXHJcbiAgICAvLyBpZiB0aGUgbmVpZ2hib3JzIG9mIHBvcnQgbm9kZXMgaGF2ZSBub3QgYmVlbiBkZXRlY3RlZCB5ZXQsIGZpbmQgdGhlbS5cclxuICAgIGlmICh0aGlzLmNvbnN1bXB0aW9uRWRnZXMubGVuZ3RoID09PSAwICYmIHRoaXMucHJvZHVjdEVkZ2VzLmxlbmd0aCA9PT0gMClcclxuICAgIHtcclxuICAgICAgICB0aGlzLmluaXRMaXN0cygpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGZpbmQgaWRlYWwgcG9zaXRpb25zXHJcbiAgICB2YXIgaW5wdXRQb3J0VGFyZ2V0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KHRydWUsIHRoaXMub3JpZW50YXRpb24pO1xyXG4gICAgdmFyIG91dHB1dFBvcnRUYXJnZXQgPSB0aGlzLmZpbmRQb3J0VGFyZ2V0UG9pbnQoZmFsc2UsIHRoaXMub3JpZW50YXRpb24pO1xyXG5cclxuICAgIGZvciAodmFyIG5vZGVJbmRleCA9IDA7IG5vZGVJbmRleCA8IHRoaXMuY29uc3VtcHRpb25FZGdlcy5sZW5ndGg7IG5vZGVJbmRleCsrKVxyXG4gICAge1xyXG4gICAgICAgIHJlc3VsdCA9IHRoaXMuY2FsY1JvdGF0aW9uYWxGb3JjZSh0cnVlLCBub2RlSW5kZXgsIGlucHV0UG9ydFRhcmdldCk7XHJcbiAgICAgICAgaWYgKE1hdGguYWJzKHJlc3VsdCkgPD0gU2JnblBEQ29uc3RhbnRzLkFOR0xFX1RPTEVSQU5DRSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvcGVyRWRnZUNvdW50Kys7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGlucHV0Um90U3VtICs9IHJlc3VsdDtcclxuICAgIH1cclxuICAgIGZvciAodmFyIG5vZGVJbmRleCA9IDA7IG5vZGVJbmRleCA8IHRoaXMucHJvZHVjdEVkZ2VzLmxlbmd0aDsgbm9kZUluZGV4KyspXHJcbiAgICB7XHJcbiAgICAgICAgcmVzdWx0ID0gdGhpcy5jYWxjUm90YXRpb25hbEZvcmNlKGZhbHNlLCBub2RlSW5kZXgsIG91dHB1dFBvcnRUYXJnZXQpO1xyXG4gICAgICAgIGlmIChNYXRoLmFicyhyZXN1bHQpIDw9IFNiZ25QRENvbnN0YW50cy5BTkdMRV9UT0xFUkFOQ0UpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLnByb3BlckVkZ2VDb3VudCsrO1xyXG4gICAgICAgIH1cclxuICAgICAgICBvdXRwdXRSb3RTdW0gKz0gcmVzdWx0O1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIG5vZGVJbmRleCA9IDA7IG5vZGVJbmRleCA8IHRoaXMuZWZmZWN0b3JFZGdlcy5sZW5ndGg7IG5vZGVJbmRleCsrKVxyXG4gICAge1xyXG4gICAgICAgIHJlc3VsdCA9IHRoaXMuY2FsY0VmZmVjdG9yQW5nbGUobm9kZUluZGV4KTtcclxuICAgICAgICBpZiAoTWF0aC5hYnMocmVzdWx0KSA8PSBTYmduUERDb25zdGFudHMuRUZGRUNUT1JfQU5HTEVfVE9MRVJBTkNFKVxyXG4gICAgICAgICAgICAgICAgdGhpcy5wcm9wZXJFZGdlQ291bnQrKztcclxuXHJcbiAgICAgICAgZWZmZWN0b3JSb3RTdW0gKz0gTWF0aC5hYnMocmVzdWx0KTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBhZGQgdG90YWwgZWZmZWN0b3Igcm90YXRpb25hbCBmb3JjZSB3aXRoIHRoZSBzYW1lIHNpZ24gb2ZcclxuICAgIC8vIHN0ZXAgc3VtIGJlY2F1c2UgaXQgZG9lcyBub3QgbWF0dGVyIGZvciBhbiBlZmZlY3RvciBub2RlXHJcbiAgICAvLyBlaXRoZXIgcm90YXRlIHRvIGxlZnQgb3IgcmlnaHQuIHRoZXJlZm9yZSBzdXBwb3J0IHRoZSByb3RhdGlvblxyXG4gICAgLy8gZGlyZWN0aW9uIG9mIGVhY2ggaXRlcmF0aW9uLlxyXG4gICAgc3RlcFN1bSA9IGlucHV0Um90U3VtIC0gb3V0cHV0Um90U3VtO1xyXG4gICAgc3RlcFN1bSA9IHN0ZXBTdW0gKyAoTWF0aC5zaWduKHN0ZXBTdW0pICogTWF0aC5hYnMoZWZmZWN0b3JSb3RTdW0pKTtcclxuXHJcbiAgICByZXR1cm4gc3RlcFN1bTtcclxufTtcclxuXHJcbi8qKlxyXG4qIFRoaXMgbWV0aG9kIHJldHVybnMgdGhlIHNpZ25lZCBhbmdsZSBiZXR3ZWVuIGEgbm9kZSBhbmQgaXRzIGNvcnJlc3BvbmRpbmdcclxuKiBwb3J0IGFuZCB0aGUgdGFyZ2V0IHBvaW50LlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmNhbGNSb3RhdGlvbmFsRm9yY2UgPSBmdW5jdGlvbiAoaXNJbnB1dFBvcnQsIG5vZGVJbmRleCwgdGFyZ2V0UG9pbnQpXHJcbntcclxuICAgIHZhciBub2RlO1xyXG4gICAgdmFyIGNlbnRlclBvaW50O1xyXG5cclxuICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgIHtcclxuICAgICAgICBub2RlID0gdGhpcy5jb25zdW1wdGlvbkVkZ2VzW25vZGVJbmRleF0uZ2V0U291cmNlKCk7XHJcbiAgICAgICAgY2VudGVyUG9pbnQgPSB0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXIoKTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICBub2RlID0gdGhpcy5wcm9kdWN0RWRnZXNbbm9kZUluZGV4XS5nZXRUYXJnZXQoKTtcclxuICAgICAgICBjZW50ZXJQb2ludCA9IHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXIoKTtcclxuICAgIH1cclxuXHJcbiAgICB2YXIgYW5nbGUgPSBJR2VvbWV0cnkuY2FsY3VsYXRlQW5nbGUodGFyZ2V0UG9pbnQsIGNlbnRlclBvaW50LCBub2RlLmdldENlbnRlcigpKTtcclxuXHJcbiAgICBpZiAoaXNJbnB1dFBvcnQpXHJcbiAgICAgICAgYW5nbGUgKj0gdGhpcy5pc0xlZnQodGFyZ2V0UG9pbnQsIGNlbnRlclBvaW50LCBub2RlLmdldENlbnRlcigpLCBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVCk7XHJcbiAgICBlbHNlXHJcbiAgICAgICAgYW5nbGUgKj0gdGhpcy5pc0xlZnQodGFyZ2V0UG9pbnQsIGNlbnRlclBvaW50LCBub2RlLmdldENlbnRlcigpLCBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpO1xyXG5cclxuICAgIHRoaXMuc2F2ZUluZm9ybWF0aW9uKGlzSW5wdXRQb3J0LCBub2RlSW5kZXgsIGFuZ2xlKTtcclxuXHJcbiAgICByZXR1cm4gYW5nbGU7XHJcbn07XHJcblxyXG4vKipcclxuKiBDYWxjdWxhdGVzIHRoZSBhbmdsZSBiZXR3ZWVuIGFuIGVmZmVjdG9yIGVkZ2UgYW5kIGl0cyBwcm9jZXNzIG5vZGUuIEFuXHJcbiogZWZmZWN0b3IgZWRnZSBoYXMgcHJvY2VzcyAoaW4gdGhpcyBjYXNlIHRoZSBkdW1teSBjb21wb3VuZCkgYXMgaXRzIHRhcmdldFxyXG4qIG5vZGUgYW5kIHRoZSBlZmZlY3RvciBpdHNlbGYgYXMgdGhlIHNvdXJjZS5cclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5jYWxjRWZmZWN0b3JBbmdsZSA9IGZ1bmN0aW9uIChub2RlSW5kZXgpXHJcbntcclxuICAgIHZhciBlZmYgPSB0aGlzLmVmZmVjdG9yRWRnZXNbbm9kZUluZGV4XS5nZXRTb3VyY2UoKTtcclxuICAgIHZhciB0YXJnZXRQbnQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICB2YXIgY2VudGVyUG50ID0gdGhpcy5nZXRDZW50ZXIoKTtcclxuXHJcbiAgICAvLyBmaW5kIHRhcmdldCBwb2ludFxyXG4gICAgaWYgKHRoaXMuaXNIb3Jpem9udGFsKCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UG50LnggPSB0aGlzLmdldENlbnRlclgoKTtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJZKCkgPiB0aGlzLmdldENlbnRlclkoKSlcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnkgPSB0aGlzLmdldENlbnRlclkoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgICAgIGVsc2VcclxuICAgICAgICAgICAgdGFyZ2V0UG50LnkgPSB0aGlzLmdldENlbnRlclkoKSAtIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAodGhpcy5pc1ZlcnRpY2FsKCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UG50LnkgPSB0aGlzLmdldENlbnRlclkoKTtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJYKCkgPiB0aGlzLmdldENlbnRlclgoKSlcclxuICAgICAgICAgICAgICAgIHRhcmdldFBudC54ID0gdGhpcy5nZXRDZW50ZXJYKCkgKyB0aGlzLmlkZWFsRWRnZUxlbmd0aDtcclxuICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgICAgICB0YXJnZXRQbnQueCA9IHRoaXMuZ2V0Q2VudGVyWCgpIC0gdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICB9XHJcblxyXG4gICAgdmFyIGFuZ2xlID0gSUdlb21ldHJ5LmNhbGN1bGF0ZUFuZ2xlKHRhcmdldFBudCwgY2VudGVyUG50LCBlZmYuZ2V0Q2VudGVyKCkpO1xyXG5cclxuICAgIHRoaXMuZWZmZWN0b3JFZGdlc1tub2RlSW5kZXhdLmNvcnJlc3BvbmRpbmdBbmdsZSA9IGFuZ2xlO1xyXG5cclxuICAgIGlmIChNYXRoLmFicyhhbmdsZSkgPD0gU2JnblBEQ29uc3RhbnRzLkVGRkVDVE9SX0FOR0xFX1RPTEVSQU5DRSlcclxuICAgICAgICB0aGlzLmVmZmVjdG9yRWRnZXNbbm9kZUluZGV4XS5pc1Byb3Blcmx5T3JpZW50ZWQgPSB0cnVlO1xyXG4gICAgZWxzZVxyXG4gICAgICAgIHRoaXMuZWZmZWN0b3JFZGdlc1tub2RlSW5kZXhdLmlzUHJvcGVybHlPcmllbnRlZCA9IGZhbHNlO1xyXG5cclxuICAgIHJldHVybiBhbmdsZTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuYXBwbHlBcHByb3hpbWF0aW9ucyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIC8vIGlmIHRoZXJlIGlzIG9ubHkgb25lIHNpbmdsZS1lZGdlIGNvbnN1bXB0aW9uLCBtb3ZlIGl0IHRvIGlkZWFsXHJcbiAgICAvLyBvdGhlcndpc2UgbW92ZSB0b3dhcmRzIG11bHRpZWRnZSBub2RlXHJcblxyXG4gICAgaWYgKHRoaXMuY29uc3VtcHRpb25FZGdlcy5sZW5ndGggPT09IDEgJiYgXHJcbiAgICAgICAgdGhpcy5jb25zdW1wdGlvbkVkZ2VzWzBdLmdldFNvdXJjZSgpLmdldEVkZ2VzKCkubGVuZ3RoID09PSAxKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuYXBwcm94aW1hdGVGb3JTaW5nbGVOb2Rlcyh0aGlzLmlucHV0UG9ydCwgdGhpcy5jb25zdW1wdGlvbkVkZ2VzWzBdLmdldFNvdXJjZSgpKTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICB0aGlzLmFwcHJveGltYXRlRm9yTXVsdGlwbGVOb2Rlcyh0aGlzLmlucHV0UG9ydCk7XHJcbiAgICB9XHJcblxyXG4gICAgaWYgKHRoaXMucHJvZHVjdEVkZ2VzLmxlbmd0aCA9PT0gMSAmJiBcclxuICAgICAgICB0aGlzLnByb2R1Y3RFZGdlc1swXS5nZXRUYXJnZXQoKS5nZXRFZGdlcygpLmxlbmd0aCA9PT0gMSlcclxuICAgIHsgICAgXHJcbiAgICAgICAgdGhpcy5hcHByb3hpbWF0ZUZvclNpbmdsZU5vZGVzKHRoaXMub3V0cHV0UG9ydCwgdGhpcy5wcm9kdWN0RWRnZXNbMF0uZ2V0VGFyZ2V0KCkpO1xyXG4gICAgfVxyXG4gICAgZWxzZVxyXG4gICAge1xyXG4gICAgICAgIGFwcHJveGltYXRlRm9yTXVsdGlwbGVOb2Rlcyh0aGlzLm91dHB1dFBvcnQpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMuYXBwcm94aW1hdGVFZmZlY3RvcnMoKTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuYXBwcm94aW1hdGVGb3JTaW5nbGVOb2RlcyA9IGZ1bmN0aW9uIChwb3J0LCBub2RlKVxyXG57XHJcbiAgICB2YXIgdGFyZ2V0UHQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICB2YXIgbmV3UG9pbnQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICBcclxuICAgIGlmIChwb3J0LmlzSW5wdXRQb3J0KCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGFyZ2V0UHQgPSB0aGlzLmZpbmRQb3J0VGFyZ2V0UG9pbnQodHJ1ZSwgdGhpcy5vcmllbnRhdGlvbik7XHJcbiAgICB9ICAgIFxyXG4gICAgXHJcbiAgICBlbHNlIGlmIChwb3J0LmlzT3V0cHV0UG9ydCgpKVxyXG4gICAge1xyXG4gICAgICAgIHRhcmdldFB0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KGZhbHNlLCB0aGlzLm9yaWVudGF0aW9uKTsgICBcclxuICAgIH1cclxuICAgIFxyXG4gICAgbmV3UG9pbnQueCA9IHRhcmdldFB0LnhcclxuICAgICAgICAgICAgICAgICAgICArIChNYXRoLnJhbmRvbSgpICogU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0UgKiAyKVxyXG4gICAgICAgICAgICAgICAgICAgIC0gU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0U7XHJcbiAgICBuZXdQb2ludC55ID0gdGFyZ2V0UHQueVxyXG4gICAgICAgICAgICAgICAgICAgICsgKE1hdGgucmFuZG9tKCkgKiBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRSAqIDIpXHJcbiAgICAgICAgICAgICAgICAgICAgLSBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRTtcclxuXHJcbiAgICBub2RlLnNldENlbnRlcihuZXdQb2ludC54LCBuZXdQb2ludC55KTtcclxufTtcclxuXHJcbi8qKlxyXG4qIEdpdmVuIHRoZSBwb3J0IG5vZGUsIHRoaXMgbWV0aG9kIGZpbmRzIGFsbCBjb25zdW1wdGlvbihvciBwcm9kdWN0aW9uKVxyXG4qIG5vZGVzIG9mIHRoaXMgcG9ydCBub2RlLiBUaGUgaWRlYSBpcyB0byBtb3ZlIGVhY2ggc2luZ2xlLWVkZ2VcclxuKiBjb25zdW1wdGlvbnMocHJvZHVjdHMpIG9mIGEgcHJvY2VzcyBjbG9zZXIgdG8gbmVpZ2hib3IgbXVsdGktZWRnZSBub2Rlc1xyXG4qIHRvIGtlZXAgdGhlbSBjbG9zZSB0byBlYWNoIG90aGVyLlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmFwcHJveGltYXRlRm9yTXVsdGlwbGVOb2RlcyA9IGZ1bmN0aW9uIChwb3J0KVxyXG57XHJcbiAgICB2YXIgb25lRWRnZU5vZGVzID0gW107XHJcbiAgICB2YXIgbXVsdGlFZGdlTm9kZXMgPSBbXTtcclxuICAgIHZhciBub2RlT2ZJbnRlcmVzdCA9IG51bGw7XHJcbiAgICB2YXIgdGFyZ2V0UHQgPSBuZXcgUG9pbnREKCk7XHJcblxyXG4gICAgLy8gZ2V0IGFsbCBub24tcmlnaWQgZWRnZXMgb2YgcG9ydCBub2RlXHJcbiAgICB2YXIgbnVtT2ZFZGdlcyA9IHBvcnQuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZFZGdlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBlZGdlID0gcG9ydC5nZXRFZGdlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAoZWRnZS5pc1JpZ2lkRWRnZSgpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgY29udGludWU7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBub2RlIG9mIGludGVyZXN0IGRlcGVuZHMgb24gdGhlIGRpcmVjdGlvbiBvZiB0aGUgZWRnZVxyXG4gICAgICAgIGlmIChwb3J0LmlzSW5wdXRQb3J0KCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBub2RlT2ZJbnRlcmVzdCA9IGVkZ2UuZ2V0U291cmNlKCk7ICAgXHJcbiAgICAgICAgfSAgICAgICAgXHJcbiAgICAgICAgZWxzZSBpZiAocG9ydC5pc091dHB1dFBvcnQoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG5vZGVPZkludGVyZXN0ID0gZWRnZS5nZXRUYXJnZXQoKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIGlmIChub2RlT2ZJbnRlcmVzdC5nZXRFZGdlcygpLmxlbmd0aCA9PT0gMSkgLy8gc2luZ2xlIG5vZGUgICAgXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBvbmVFZGdlTm9kZXMucHVzaChub2RlT2ZJbnRlcmVzdCk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKG5vZGVPZkludGVyZXN0LmdldEVkZ2VzKCkubGVuZ3RoID4gMSkgLy8gbXVsdGllZGdlIG5vZGVcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIG11bHRpRWRnZU5vZGVzLnB1c2gobm9kZU9mSW50ZXJlc3QpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICBpZiAocG9ydC5pc0lucHV0UG9ydCgpKVxyXG4gICAge1xyXG4gICAgICAgIHRhcmdldFB0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KHRydWUsIHRoaXMub3JpZW50YXRpb24pOyAgIFxyXG4gICAgfSAgICAgICAgXHJcbiAgICBlbHNlIGlmIChwb3J0LmlzT3V0cHV0UG9ydCgpKVxyXG4gICAge1xyXG4gICAgICAgIHRhcmdldFB0ID0gdGhpcy5maW5kUG9ydFRhcmdldFBvaW50KGZhbHNlLCB0aGlzLm9yaWVudGF0aW9uKTtcclxuICAgIH0gICAgICAgIFxyXG4gICAgXHJcbiAgICAvLyBtb3ZlXHJcbiAgICBpZiAob25lRWRnZU5vZGVzLmxlbmd0aCA+IDApXHJcbiAgICB7XHJcbiAgICAgICAgbW92ZU9uZUVkZ2VOb2RlcyhvbmVFZGdlTm9kZXMsIG11bHRpRWRnZU5vZGVzLCB0YXJnZXRQdCk7XHJcbiAgICB9ICAgIFxyXG59O1xyXG5cclxuLyoqXHJcbiogU2luZ2xlLWVkZ2Ugbm9kZXMgYXJlIG1vdmVkIGFyb3VuZCB0aGUgY2VudGVyIHBvaW50IG9mIGEgbXVsdGktZWRnZSBub2RlLlxyXG4qIElmIGFsbCB0aGUgbmVpZ2hib3Igbm9kZXMgb2YgdGhhdCBwb3J0IG5vZGUgYXJlIHNpbmdsZS1lZGdlZCwgdGhlbiBvbmUgb2ZcclxuKiB0aGVtIGlzIGNob3NlbiByYW5kb21seSBhbmQgdGhlIG90aGVycyBhcmUgcGxhY2VkIGFyb3VuZCBpdC5cclxuKiBcclxuKiBAcGFyYW0gdGFyZ2V0UHRcclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5tb3ZlT25lRWRnZU5vZGVzID0gZnVuY3Rpb24gKG9uZUVkZ2VOb2RlcywgbXVsdGlFZGdlTm9kZXMsIHRhcmdldFB0KVxyXG57XHJcbiAgICB2YXIgYXBwcm94aW1hdGlvblBudCA9IG5ldyBQb2ludEQoMCwgMCk7XHJcbiAgICB2YXIgcmFuZG9tSW5kZXggPSAtMTtcclxuICAgIHZhciBhcHByb3hpbWF0aW9uTm9kZSA9IG51bGw7XHJcbiAgICB2YXIgbmV3UG9pbnQgPSBuZXcgUG9pbnREKCk7XHJcblxyXG4gICAgLy8gaWYgdGhlcmUgYXJlIG1vcmUgdGhhbiBvbmUgbXVsdGkgZWRnZSBub2Rlcywgc2VsZWN0IHRoZSBoaWdobHlcclxuICAgIC8vIGNvbm5lY3RlZCBvbmVcclxuICAgIGlmIChtdWx0aUVkZ2VOb2Rlcy5sZW5ndGggPiAwKVxyXG4gICAge1xyXG4gICAgICAgIGFwcHJveGltYXRpb25Ob2RlID0gbXVsdGlFZGdlTm9kZXNbMF07XHJcbiAgICAgICAgXHJcbiAgICAgICAgdmFyIG51bU9mTXVsdGlFZGdlTm9kZXMgPSBtdWx0aUVkZ2VOb2Rlcy5sZW5ndGg7XHJcbiAgICAgICAgZm9yICh2YXIgaT0wOyBpPG51bU9mTXVsdGlFZGdlTm9kZXM7IGkrKylcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciBub2RlID0gbXVsdGlFZGdlTm9kZXNbaV07XHJcblxyXG4gICAgICAgICAgICBpZiAobm9kZS5nZXRFZGdlcygpLmxlbmd0aCA+IGFwcHJveGltYXRpb25Ob2RlLmdldEVkZ2VzKCkubGVuZ3RoKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBhcHByb3hpbWF0aW9uTm9kZSA9IG5vZGU7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gaWYgdGhlcmUgYXJlIG5vIG11bHRpIGVkZ2Ugbm9kZXMsIHJhbmRvbWx5IHNlbGVjdCBvbmVcclxuICAgIGVsc2UgaWYgKG11bHRpRWRnZU5vZGVzLmxlbmd0aCA9PT0gMClcclxuICAgIHtcclxuICAgICAgICByYW5kb21JbmRleCA9IChNYXRoLnJhbmRvbSgpICogb25lRWRnZU5vZGVzLmxlbmd0aCk7XHJcbiAgICAgICAgYXBwcm94aW1hdGlvbk5vZGUgPSBvbmVFZGdlTm9kZXNbcmFuZG9tSW5kZXhdO1xyXG4gICAgfVxyXG5cclxuICAgIGFwcHJveGltYXRpb25QbnQgPSBhcHByb3hpbWF0aW9uTm9kZS5nZXRDZW50ZXIoKTtcclxuXHJcbiAgICAvLyBtb3ZlIHNpbmdsZSBub2RlcyBhcm91bmQgdGhlIGFwcHJveGltYXRpb24gcG9pbnRcclxuICAgIHZhciBudW1PZk9uZUVkZ2VOb2RlcyA9IG9uZUVkZ2VOb2Rlcy5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZPbmVFZGdlTm9kZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgcyA9IG9uZUVkZ2VOb2Rlc1tpXTtcclxuICAgICAgICBcclxuICAgICAgICAvLyBpZiB0aGV5IGJlbG9uZyB0byBkaWZmZXJlbnQgZ3JhcGhzLCBkbyBub3QgbW92ZVxyXG4gICAgICAgIC8vIGlmIChhcHByb3hpbWF0aW9uTm9kZS5nZXRPd25lcigpICE9IHMuZ2V0T3duZXIoKSlcclxuICAgICAgICAvLyBjb250aW51ZTtcclxuICAgICAgICBuZXdQb2ludC54ID0gYXBwcm94aW1hdGlvblBudC54XHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgKE1hdGgucmFuZG9tKCkgKiBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRSAqIDIpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgIC0gU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0U7XHJcbiAgICAgICAgbmV3UG9pbnQueSA9IGFwcHJveGltYXRpb25QbnQueVxyXG4gICAgICAgICAgICAgICAgICAgICAgICArIChNYXRoLnJhbmRvbSgpICogU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0UgKiAyKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX0RJU1RBTkNFO1xyXG5cclxuICAgICAgICBzLnNldENlbnRlcihuZXdQb2ludC54LCBuZXdQb2ludC55KTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIElkZW50aWZ5IHNpbmdsZS1lZGdlIGVmZmVjdG9ycy4gTm90ZSB0aGF0IGEgcHJvY2VzcyBtYXkgaGF2ZSBhIG51bWJlciBvZlxyXG4qIGVmZmVjdG9ycy4gRmluZCB0aGUgbG9jYXRpb24gb2YgZWFjaCBlZmZlY3Rvci4gSWYgdGhlIG9yaWVudGF0aW9uIG9mXHJcbiogcHJvY2VzcyBub2RlIGlzIHZlcnRpY2FsLCBpZGVhbCBwb3NpdGlvbiBvZiBlZmZlY3RvcnMgaXMgb24gdGhlXHJcbiogaG9yaXpvbnRhbCBkaXJlY3Rpb25zLiAob3IgdmljZSB2ZXJzYSlcclxuKi9cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5hcHByb3hpbWF0ZUVmZmVjdG9ycyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBuZXdQb2ludCA9IG5ldyBQb2ludEQoKTtcclxuICAgIHZhciBhcHByb3hpbWF0aW9uUG50ID0gbmV3IFBvaW50RCgpO1xyXG5cclxuICAgIC8vIGlkZW50aWZ5IHRoZSBlZmZlY3RvcnNcclxuICAgIHZhciBudW1PZkVmZmVjdG9yRWRnZXMgPSB0aGlzLmVmZmVjdG9yRWRnZXMubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaT0wOyBpPG51bU9mRWZmZWN0b3JFZGdlczsgaSsrKVxyXG4gICAge1xyXG4gICAgICAgIHZhciBlZGdlID0gdGhpcy5lZmZlY3RvckVkZ2VzW2ldO1xyXG4gICAgICAgIFxyXG4gICAgICAgIC8vIHNvdXJjZSBub2RlIG9mIGVhY2ggZWZmZWN0b3IgZWRnZSBpcyB0aGUgZWZmZWN0b3IgaXRzZWxmLiBvbmx5XHJcbiAgICAgICAgLy8gbW92ZSBzaW5nbGUgZWZmZWN0b3JzXHJcbiAgICAgICAgaWYgKGVkZ2UuZ2V0U291cmNlKCkuZ2V0RWRnZXMoKS5sZW5ndGggIT09IDEpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBjb250aW51ZTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIGFwcHJveGltYXRpb25QbnQgPSB0aGlzLmZpbmRFZmZlY3RvclRhcmdldFBvaW50KGVkZ2UuZ2V0U291cmNlKCkpO1xyXG5cclxuICAgICAgICAvLyBwbGFjZSBlZmZlY3RvciBpbiBhIGNpcmN1bGFyIGFyZWEgdXNpbmcgc29tZSByYW5kb21uZXNzXHJcbiAgICAgICAgbmV3UG9pbnQueCA9IGFwcHJveGltYXRpb25QbnQueFxyXG4gICAgICAgICAgICAgICAgICAgICAgICArIChNYXRoLnJhbmRvbSgpICogU2JnblBEQ29uc3RhbnRzLkFQUFJPWElNQVRJT05fRElTVEFOQ0UgKiAyKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX0RJU1RBTkNFO1xyXG4gICAgICAgIG5ld1BvaW50LnkgPSBhcHByb3hpbWF0aW9uUG50LnlcclxuICAgICAgICAgICAgICAgICAgICAgICAgKyAoTWF0aC5yYW5kb20oKSAqIFNiZ25QRENvbnN0YW50cy5BUFBST1hJTUFUSU9OX0RJU1RBTkNFICogMilcclxuICAgICAgICAgICAgICAgICAgICAgICAgLSBTYmduUERDb25zdGFudHMuQVBQUk9YSU1BVElPTl9ESVNUQU5DRTtcclxuXHJcbiAgICAgICAgZWRnZS5nZXRTb3VyY2UoKS5zZXRDZW50ZXIobmV3UG9pbnQueCwgbmV3UG9pbnQueSk7XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBHaXZlbiB0aGUgZWZmZWN0b3IgYW5kIGl0cyBjb3JyZXNwb25kaW5nIHByb2Nlc3MsIHRoZSBtZXRob2QgcmV0dXJucyB0aGVcclxuKiBpZGVhbCBwb3NpdGlvbiBvZiB0aGUgZWZmZWN0b3Igbm9kZSwgd2hpY2ggaGFzIGEgZGlzdGFuY2Ugb2YgaWRlYWwgZWRnZVxyXG4qIGxlbmd0aCBmcm9tIGl0cyBwcm9jZXNzLlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmZpbmRFZmZlY3RvclRhcmdldFBvaW50ID0gZnVuY3Rpb24gKGVmZilcclxue1xyXG4gICAgdmFyIGFwcHJveGltYXRpb25QbnQgPSBuZXcgUG9pbnREKCk7XHJcbiAgICBcclxuICAgIC8vIGZpbmQgdGFyZ2V0IHBvaW50XHJcbiAgICBpZiAodGhpcy5pc0hvcml6b250YWwoKSlcclxuICAgIHtcclxuICAgICAgICBhcHByb3hpbWF0aW9uUG50LnggPSB0aGlzLmdldENlbnRlclgoKTtcclxuXHJcbiAgICAgICAgaWYgKGVmZi5nZXRDZW50ZXJZKCkgPiB0aGlzLmdldENlbnRlclkoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGFwcHJveGltYXRpb25QbnQueSA9IHRoaXMuZ2V0Q2VudGVyWSgpICsgdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGFwcHJveGltYXRpb25QbnQueSA9IHRoaXMuZ2V0Q2VudGVyWSgpIC0gdGhpcy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAodGhpcy5pc1ZlcnRpY2FsKCkpXHJcbiAgICB7XHJcbiAgICAgICAgYXBwcm94aW1hdGlvblBudC55ID0gdGhpcy5nZXRDZW50ZXJZKCk7XHJcblxyXG4gICAgICAgIGlmIChlZmYuZ2V0Q2VudGVyWCgpID4gdGhpcy5nZXRDZW50ZXJYKCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBhcHByb3hpbWF0aW9uUG50LnggPSB0aGlzLmdldENlbnRlclgoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBhcHByb3hpbWF0aW9uUG50LnggPSB0aGlzLmdldENlbnRlclgoKSAtIHRoaXMuaWRlYWxFZGdlTGVuZ3RoO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gYXBwcm94aW1hdGlvblBudDtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuZmluZFBvcnRUYXJnZXRQb2ludCA9IGZ1bmN0aW9uIChpc0lucHV0UG9ydCwgb3JpZW50KVxyXG57XHJcbiAgICBpZiAob3JpZW50ID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKCh0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXJYKCkgLSB0aGlzLmlkZWFsRWRnZUxlbmd0aCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclkoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKCh0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWCgpICsgdGhpcy5pZGVhbEVkZ2VMZW5ndGgpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICB0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWSgpKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmIChvcmllbnQgPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKGlzSW5wdXRQb3J0KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBQb2ludEQoKHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclgoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgdGhpcy5pbnB1dFBvcnQuZ2V0Q2VudGVyWSgpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBQb2ludEQoKHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXJYKCkgLSB0aGlzLmlkZWFsRWRnZUxlbmd0aCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgIHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXJZKCkpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIGVsc2UgaWYgKG9yaWVudCA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uVE9QX1RPX0JPVFRPTSlcclxuICAgIHtcclxuICAgICAgICBpZiAoaXNJbnB1dFBvcnQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXJYKCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICh0aGlzLmlucHV0UG9ydC5nZXRDZW50ZXJZKCkgLSB0aGlzLmlkZWFsRWRnZUxlbmd0aCkpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFBvaW50RCh0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWCgpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAodGhpcy5vdXRwdXRQb3J0LmdldENlbnRlclkoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAob3JpZW50ID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5CT1RUT01fVE9fVE9QKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclgoKSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgKHRoaXMuaW5wdXRQb3J0LmdldENlbnRlclkoKSArIHRoaXMuaWRlYWxFZGdlTGVuZ3RoKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBuZXcgUG9pbnREKHRoaXMub3V0cHV0UG9ydC5nZXRDZW50ZXJYKCksXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICh0aGlzLm91dHB1dFBvcnQuZ2V0Q2VudGVyWSgpIC0gdGhpcy5pZGVhbEVkZ2VMZW5ndGgpKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIG51bGw7XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLmluaXRMaXN0cyA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIHZhciBudW1PZkNvbnN1bXB0aW9uRWRnZXMgPSB0aGlzLmlucHV0UG9ydC5nZXRFZGdlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkNvbnN1bXB0aW9uRWRnZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMuaW5wdXRQb3J0LmdldEVkZ2VzKClbaV07XHJcbiAgICAgICAgXHJcbiAgICAgICAgaWYgKCFlZGdlLmlzUmlnaWRFZGdlKCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB0aGlzLmNvbnN1bXB0aW9uRWRnZXMuYWRkKGVkZ2UpO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuXHJcbiAgICB2YXIgbnVtT2ZQcm9kdWN0RWRnZXMgPSB0aGlzLm91dHB1dFBvcnQuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZQcm9kdWN0RWRnZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZWRnZSA9IHRoaXMub3V0cHV0UG9ydC5nZXRFZGdlcygpW2ldO1xyXG5cclxuICAgICAgICBpZiAoIWVkZ2UuaXNSaWdpZEVkZ2UoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvZHVjdEVkZ2VzLmFkZChlZGdlKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgLy8gZGV0ZWN0IGFsbCBlZmZlY3RvciBub2RlcyBjb25uZWN0ZWQgdG8gdGhpcyBwcm9jZXNzIG5vZGUgKGlmXHJcbiAgICAvLyB0aGVyZSBhcmUgYW55KVxyXG4gICAgdmFyIG51bU9mRWZmZWN0b3JFZGdlcyA9IHRoaXMucGFyZW50Q29tcG91bmQuZ2V0RWRnZXMoKS5sZW5ndGg7XHJcbiAgICBmb3IgKHZhciBpPTA7IGk8bnVtT2ZFZmZlY3RvckVkZ2VzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSB0aGlzLnBhcmVudENvbXBvdW5kLmdldEVkZ2VzKClbaV07XHJcblxyXG4gICAgICAgIGlmIChlZGdlLmlzRWZmZWN0b3IoKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuZWZmZWN0b3JFZGdlcy5hZGQoZWRnZSk7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5zYXZlSW5mb3JtYXRpb24gPSBmdW5jdGlvbiAoaXNJbnB1dFBvcnQsIG5vZGVJbmRleCwgYW5nbGUpXHJcbntcclxuXHJcbiAgICAvLyByZW1lbWJlciBhbmdsZXMgZXNwZWNpYWxseSBmb3IgZGVidWcgcHVycG9zZXNcclxuICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgIHtcclxuICAgICAgICB0aGlzLmNvbnN1bXB0aW9uRWRnZXNbbm9kZUluZGV4XS5jb3JyZXNwb25kaW5nQW5nbGUgPSBhbmdsZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICB0aGlzLnByb2R1Y3RFZGdlc1tub2RlSW5kZXhdLmNvcnJlc3BvbmRpbmdBbmdsZSA9IGFuZ2xlO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIG5vdGUgaWYgdGhlIGVkZ2VzIGFyZSBwcm9wZXJseSBvcmllbnRlZFxyXG4gICAgaWYgKE1hdGguYWJzKGFuZ2xlKSA8PSBTYmduUERDb25zdGFudHMuQU5HTEVfVE9MRVJBTkNFKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChpc0lucHV0UG9ydClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMuY29uc3VtcHRpb25FZGdlc1tub2RlSW5kZXhdLmlzUHJvcGVybHlPcmllbnRlZCA9IHRydWU7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvZHVjdEVkZ2VzW25vZGVJbmRleF0uaXNQcm9wZXJseU9yaWVudGVkID0gdHJ1ZTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKGlzSW5wdXRQb3J0KVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdGhpcy5jb25zdW1wdGlvbkVkZ2VzW25vZGVJbmRleF0uaXNQcm9wZXJseU9yaWVudGVkID0gZmFsc2U7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHRoaXMucHJvZHVjdEVkZ2VzW25vZGVJbmRleF0uaXNQcm9wZXJseU9yaWVudGVkID0gZmFsc2U7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5pc0xlZnQgPSBmdW5jdGlvbiAoYSwgYiwgYywgdHlwZSlcclxue1xyXG4gICAgaWYgKCgoYi54IC0gYS54KSAqIChjLnkgLSBhLnkpIC0gKGIueSAtIGEueSkgKiAoYy54IC0gYS54KSkgPiAwKVxyXG4gICAge1xyXG4gICAgICAgIC8vIGxlZnQgdHVyblxyXG4gICAgICAgIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIC0xO1xyXG4gICAgICAgICAgICBlbHNlIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gMTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAxO1xyXG4gICAgICAgICAgICBlbHNlIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gLTE7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLklOUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gMTtcclxuICAgICAgICAgICAgZWxzZSBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLk9VVFBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIC0xO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIC0xO1xyXG4gICAgICAgICAgICBlbHNlIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuT1VUUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gMTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICBlbHNlXHJcbiAgICB7XHJcbiAgICAgICAgLy8gcmlnaHQgdHVyblxyXG4gICAgICAgIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDE7XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAtMTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIGlmICh0eXBlID09PSBTYmduUERDb25zdGFudHMuSU5QVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAtMTtcclxuICAgICAgICAgICAgZWxzZSBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLk9VVFBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDE7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodHlwZSA9PT0gU2JnblBEQ29uc3RhbnRzLklOUFVUX1BPUlQpXHJcbiAgICAgICAgICAgICAgICByZXR1cm4gLTE7XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAxO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5SSUdIVF9UT19MRUZUKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5JTlBVVF9QT1JUKVxyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDE7XHJcbiAgICAgICAgICAgIGVsc2UgaWYgKHR5cGUgPT09IFNiZ25QRENvbnN0YW50cy5PVVRQVVRfUE9SVClcclxuICAgICAgICAgICAgICAgIHJldHVybiAtMTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICByZXR1cm4gMDtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuY29weU5vZGUgPSBmdW5jdGlvbiAocywgZ3JhcGhNYW5hZ2VyKSAvLyBUT0RPOiBncmFwaE1hbmFnZXIgaXMgbm90IHVzZWQuXHJcbntcclxuICAgIHRoaXMudHlwZSA9IHMudHlwZTtcclxuICAgIHRoaXMubGFiZWwgPSBzLmxhYmVsO1xyXG4gICAgdGhpcy5wYXJlbnRDb21wb3VuZCA9IHMucGFyZW50Q29tcG91bmQ7XHJcbiAgICB0aGlzLmlucHV0UG9ydCA9IHMuaW5wdXRQb3J0O1xyXG4gICAgdGhpcy5vdXRwdXRQb3J0ID0gcy5vdXRwdXRQb3J0O1xyXG4gICAgdGhpcy5zZXRDZW50ZXIocy5nZXRDZW50ZXJYKCksIHMuZ2V0Q2VudGVyWSgpKTtcclxuICAgIHRoaXMuc2V0Q2hpbGQocy5nZXRDaGlsZCgpKTtcclxuICAgIHRoaXMuc2V0SGVpZ2h0KHMuZ2V0SGVpZ2h0KCkpO1xyXG4gICAgdGhpcy5zZXRMb2NhdGlvbihzLmdldExvY2F0aW9uKCkueCwgcy5nZXRMb2NhdGlvbigpLnkpO1xyXG4gICAgdGhpcy5zZXROZXh0KHMuZ2V0TmV4dCgpKTtcclxuICAgIHRoaXMuc2V0T3duZXIocy5nZXRPd25lcigpKTtcclxuICAgIHRoaXMuc2V0UHJlZDEocy5nZXRQcmVkMSgpKTtcclxuICAgIHRoaXMuc2V0UHJlZDIocy5nZXRQcmVkMigpKTtcclxuICAgIHRoaXMuc2V0V2lkdGgocy5nZXRXaWR0aCgpKTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuY29weUZyb21TQkdOUEROb2RlID0gZnVuY3Rpb24gKHMsIGdyYXBoTWFuYWdlcilcclxue1xyXG4gICAgdGhpcy50eXBlID0gcy50eXBlO1xyXG4gICAgdGhpcy5sYWJlbCA9IHMubGFiZWw7XHJcbiAgICB0aGlzLnNldENlbnRlcihzLmdldENlbnRlclgoKSwgcy5nZXRDZW50ZXJZKCkpO1xyXG4gICAgdGhpcy5zZXRDaGlsZChzLmdldENoaWxkKCkpO1xyXG4gICAgdGhpcy5zZXRIZWlnaHQocy5nZXRIZWlnaHQoKSk7XHJcbiAgICB0aGlzLnNldExvY2F0aW9uKHMuZ2V0TG9jYXRpb24oKS54LCBzLmdldExvY2F0aW9uKCkueSk7XHJcbiAgICB0aGlzLnNldE5leHQocy5nZXROZXh0KCkpO1xyXG4gICAgdGhpcy5zZXRPd25lcihzLmdldE93bmVyKCkpO1xyXG4gICAgdGhpcy5zZXRQcmVkMShzLmdldFByZWQxKCkpO1xyXG4gICAgdGhpcy5zZXRQcmVkMihzLmdldFByZWQyKCkpO1xyXG4gICAgdGhpcy5zZXRXaWR0aChzLmdldFdpZHRoKCkpO1xyXG5cclxuICAgIC8vIGNvcHkgZWRnZXNcclxuICAgIHZhciBudW1PZkVkZ2VzID0gcy5nZXRFZGdlcygpLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGk9MDsgaTxudW1PZkVkZ2VzOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSBzLmdldEVkZ2VzKClbaV07XHJcbiAgICAgICAgdmFyIG5ld0VkZ2UgPSBuZXcgU2JnblBERWRnZShlZGdlLmdldFNvdXJjZSgpLCBlZGdlLmdldFRhcmdldCgpLCBudWxsLCBlZGdlLnR5cGUpO1xyXG5cclxuICAgICAgICBuZXdFZGdlLmNvcHkoZWRnZSk7XHJcblxyXG4gICAgICAgIGlmIChlZGdlLmdldFNvdXJjZSgpID09PSBzKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgbmV3RWRnZS5zZXRTb3VyY2UodGhpcyk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2UgaWYgKGVkZ2UuZ2V0VGFyZ2V0KCkgPT09IHMpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBuZXdFZGdlLnNldFRhcmdldCh0aGlzKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIC8vIGFkZCBuZXcgZWRnZSB0byB0aGUgZ3JhcGggbWFuYWdlci5cclxuICAgICAgICBncmFwaE1hbmFnZXIuYWRkKG5ld0VkZ2UsIG5ld0VkZ2UuZ2V0U291cmNlKCksIG5ld0VkZ2UuZ2V0VGFyZ2V0KCkpO1xyXG4gICAgfVxyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5pc1ZlcnRpY2FsID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlRPUF9UT19CT1RUT00gfHwgXHJcbiAgICAgICAgdGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7ICAgXHJcbiAgICB9XHJcbiAgICBcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuaXNIb3Jpem9udGFsID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLkxFRlRfVE9fUklHSFQgfHwgXHJcbiAgICAgICAgdGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uUklHSFRfVE9fTEVGVClcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuICAgIGVsc2VcclxuICAgIHtcclxuICAgICAgICByZXR1cm4gZmFsc2U7XHJcbiAgICB9XHJcbn07XHJcblxyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLnNldE9yaWVudGF0aW9uID0gZnVuY3Rpb24gKG9yaWVudClcclxue1xyXG4gICAgdGhpcy5vcmllbnRhdGlvbiA9IG9yaWVudDtcclxuICAgIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5MRUZUX1RPX1JJR0hUKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuaW5wdXRQb3J0LnNldENlbnRlcih0aGlzLmdldENlbnRlclgoKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFX0xFTkdUSCwgdGhpcy5nZXRDZW50ZXJZKCkpO1xyXG4gICAgICAgIHRoaXMub3V0cHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKClcclxuICAgICAgICAgICAgICAgICAgICAgICAgKyBTYmduUERDb25zdGFudHMuUklHSURfRURHRV9MRU5HVEgsIHRoaXMuZ2V0Q2VudGVyWSgpKTtcclxuICAgIH1cclxuICAgIGVsc2UgaWYgKHRoaXMub3JpZW50YXRpb24gPT09IHRoaXMuT3JpZW50YXRpb25FbnVtLlJJR0hUX1RPX0xFRlQpXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5pbnB1dFBvcnQuc2V0Q2VudGVyKHRoaXMuZ2V0Q2VudGVyWCgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RILCB0aGlzLmdldENlbnRlclkoKSk7XHJcbiAgICAgICAgdGhpcy5vdXRwdXRQb3J0LnNldENlbnRlcih0aGlzLmdldENlbnRlclgoKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFX0xFTkdUSCwgdGhpcy5nZXRDZW50ZXJZKCkpO1xyXG4gICAgfVxyXG4gICAgZWxzZSBpZiAodGhpcy5vcmllbnRhdGlvbiA9PT0gdGhpcy5PcmllbnRhdGlvbkVudW0uQk9UVE9NX1RPX1RPUClcclxuICAgIHtcclxuICAgICAgICB0aGlzLmlucHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKCksIHRoaXMuZ2V0Q2VudGVyWSgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RIKTtcclxuICAgICAgICB0aGlzLm91dHB1dFBvcnQuc2V0Q2VudGVyKHRoaXMuZ2V0Q2VudGVyWCgpLCB0aGlzLmdldENlbnRlclkoKVxyXG4gICAgICAgICAgICAgICAgICAgICAgICAtIFNiZ25QRENvbnN0YW50cy5SSUdJRF9FREdFX0xFTkdUSCk7XHJcbiAgICB9XHJcbiAgICBlbHNlIGlmICh0aGlzLm9yaWVudGF0aW9uID09PSB0aGlzLk9yaWVudGF0aW9uRW51bS5UT1BfVE9fQk9UVE9NKVxyXG4gICAge1xyXG4gICAgICAgIHRoaXMuaW5wdXRQb3J0LnNldENlbnRlcih0aGlzLmdldENlbnRlclgoKSwgdGhpcy5nZXRDZW50ZXJZKClcclxuICAgICAgICAgICAgICAgICAgICAgICAgLSBTYmduUERDb25zdGFudHMuUklHSURfRURHRV9MRU5HVEgpO1xyXG4gICAgICAgIHRoaXMub3V0cHV0UG9ydC5zZXRDZW50ZXIodGhpcy5nZXRDZW50ZXJYKCksIHRoaXMuZ2V0Q2VudGVyWSgpXHJcbiAgICAgICAgICAgICAgICAgICAgICAgICsgU2JnblBEQ29uc3RhbnRzLlJJR0lEX0VER0VfTEVOR1RIKTtcclxuICAgIH1cclxufTtcclxuXHJcbi8qKlxyXG4qIFRyYW5zZmVyIGZvcmNlcyBhY3Rpbmcgb24gcHJvY2VzcyBub2RlIHRvIGl0cyBwYXJlbnQgY29tcG91bmQgdG8gbWFrZVxyXG4qIHN1cmUgcHJvY2VzcyBkb2VzIG5vdCBtb3ZlLlxyXG4qL1xyXG5TYmduUHJvY2Vzc05vZGUucHJvdG90eXBlLnRyYW5zZmVyRm9yY2VzID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgdGhpcy5wYXJlbnRDb21wb3VuZC5zcHJpbmdGb3JjZVggKz0gdGhpcy5zcHJpbmdGb3JjZVhcclxuICAgICAgICAgICAgICAgICAgICArIHRoaXMuaW5wdXRQb3J0LnNwcmluZ0ZvcmNlWCArIHRoaXMub3V0cHV0UG9ydC5zcHJpbmdGb3JjZVg7XHJcbiAgICB0aGlzLnBhcmVudENvbXBvdW5kLnNwcmluZ0ZvcmNlWSArPSB0aGlzLnNwcmluZ0ZvcmNlWVxyXG4gICAgICAgICAgICAgICAgICAgICsgdGhpcy5pbnB1dFBvcnQuc3ByaW5nRm9yY2VZICsgdGhpcy5vdXRwdXRQb3J0LnNwcmluZ0ZvcmNlWTtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuZ2V0SW5wdXRQb3J0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMuaW5wdXRQb3J0O1xyXG59O1xyXG5cclxuU2JnblByb2Nlc3NOb2RlLnByb3RvdHlwZS5nZXRPdXRwdXRQb3J0ID0gZnVuY3Rpb24gKClcclxue1xyXG4gICAgcmV0dXJuIHRoaXMub3V0cHV0UG9ydDtcclxufTtcclxuXHJcblNiZ25Qcm9jZXNzTm9kZS5wcm90b3R5cGUuZ2V0UGFyZW50Q29tcG91bmQgPSBmdW5jdGlvbiAoKVxyXG57XHJcbiAgICByZXR1cm4gcGFyZW50Q29tcG91bmQ7XHJcbn07XHJcblxyXG5tb2R1bGUuZXhwb3J0cyA9IFNiZ25Qcm9jZXNzTm9kZTtcclxuIiwiZnVuY3Rpb24gVHJhbnNmb3JtKHgsIHkpIHtcbiAgdGhpcy5sd29ybGRPcmdYID0gMC4wO1xuICB0aGlzLmx3b3JsZE9yZ1kgPSAwLjA7XG4gIHRoaXMubGRldmljZU9yZ1ggPSAwLjA7XG4gIHRoaXMubGRldmljZU9yZ1kgPSAwLjA7XG4gIHRoaXMubHdvcmxkRXh0WCA9IDEuMDtcbiAgdGhpcy5sd29ybGRFeHRZID0gMS4wO1xuICB0aGlzLmxkZXZpY2VFeHRYID0gMS4wO1xuICB0aGlzLmxkZXZpY2VFeHRZID0gMS4wO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkT3JnWCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZE9yZ1g7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRPcmdYID0gZnVuY3Rpb24gKHdveClcbntcbiAgdGhpcy5sd29ybGRPcmdYID0gd294O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkT3JnWSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZE9yZ1k7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRPcmdZID0gZnVuY3Rpb24gKHdveSlcbntcbiAgdGhpcy5sd29ybGRPcmdZID0gd295O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkRXh0WCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZEV4dFg7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRFeHRYID0gZnVuY3Rpb24gKHdleClcbntcbiAgdGhpcy5sd29ybGRFeHRYID0gd2V4O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldFdvcmxkRXh0WSA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmx3b3JsZEV4dFk7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0V29ybGRFeHRZID0gZnVuY3Rpb24gKHdleSlcbntcbiAgdGhpcy5sd29ybGRFeHRZID0gd2V5O1xufVxuXG4vKiBEZXZpY2UgcmVsYXRlZCAqL1xuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldERldmljZU9yZ1ggPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sZGV2aWNlT3JnWDtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5zZXREZXZpY2VPcmdYID0gZnVuY3Rpb24gKGRveClcbntcbiAgdGhpcy5sZGV2aWNlT3JnWCA9IGRveDtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5nZXREZXZpY2VPcmdZID0gZnVuY3Rpb24gKClcbntcbiAgcmV0dXJuIHRoaXMubGRldmljZU9yZ1k7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuc2V0RGV2aWNlT3JnWSA9IGZ1bmN0aW9uIChkb3kpXG57XG4gIHRoaXMubGRldmljZU9yZ1kgPSBkb3k7XG59XG5cblRyYW5zZm9ybS5wcm90b3R5cGUuZ2V0RGV2aWNlRXh0WCA9IGZ1bmN0aW9uICgpXG57XG4gIHJldHVybiB0aGlzLmxkZXZpY2VFeHRYO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLnNldERldmljZUV4dFggPSBmdW5jdGlvbiAoZGV4KVxue1xuICB0aGlzLmxkZXZpY2VFeHRYID0gZGV4O1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmdldERldmljZUV4dFkgPSBmdW5jdGlvbiAoKVxue1xuICByZXR1cm4gdGhpcy5sZGV2aWNlRXh0WTtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5zZXREZXZpY2VFeHRZID0gZnVuY3Rpb24gKGRleSlcbntcbiAgdGhpcy5sZGV2aWNlRXh0WSA9IGRleTtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS50cmFuc2Zvcm1YID0gZnVuY3Rpb24gKHgpXG57XG4gIHZhciB4RGV2aWNlID0gMC4wO1xuICB2YXIgd29ybGRFeHRYID0gdGhpcy5sd29ybGRFeHRYO1xuICBpZiAod29ybGRFeHRYICE9IDAuMClcbiAge1xuICAgIHhEZXZpY2UgPSB0aGlzLmxkZXZpY2VPcmdYICtcbiAgICAgICAgICAgICgoeCAtIHRoaXMubHdvcmxkT3JnWCkgKiB0aGlzLmxkZXZpY2VFeHRYIC8gd29ybGRFeHRYKTtcbiAgfVxuXG4gIHJldHVybiB4RGV2aWNlO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLnRyYW5zZm9ybVkgPSBmdW5jdGlvbiAoeSlcbntcbiAgdmFyIHlEZXZpY2UgPSAwLjA7XG4gIHZhciB3b3JsZEV4dFkgPSB0aGlzLmx3b3JsZEV4dFk7XG4gIGlmICh3b3JsZEV4dFkgIT0gMC4wKVxuICB7XG4gICAgeURldmljZSA9IHRoaXMubGRldmljZU9yZ1kgK1xuICAgICAgICAgICAgKCh5IC0gdGhpcy5sd29ybGRPcmdZKSAqIHRoaXMubGRldmljZUV4dFkgLyB3b3JsZEV4dFkpO1xuICB9XG5cblxuICByZXR1cm4geURldmljZTtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5pbnZlcnNlVHJhbnNmb3JtWCA9IGZ1bmN0aW9uICh4KVxue1xuICB2YXIgeFdvcmxkID0gMC4wO1xuICB2YXIgZGV2aWNlRXh0WCA9IHRoaXMubGRldmljZUV4dFg7XG4gIGlmIChkZXZpY2VFeHRYICE9IDAuMClcbiAge1xuICAgIHhXb3JsZCA9IHRoaXMubHdvcmxkT3JnWCArXG4gICAgICAgICAgICAoKHggLSB0aGlzLmxkZXZpY2VPcmdYKSAqIHRoaXMubHdvcmxkRXh0WCAvIGRldmljZUV4dFgpO1xuICB9XG5cblxuICByZXR1cm4geFdvcmxkO1xufVxuXG5UcmFuc2Zvcm0ucHJvdG90eXBlLmludmVyc2VUcmFuc2Zvcm1ZID0gZnVuY3Rpb24gKHkpXG57XG4gIHZhciB5V29ybGQgPSAwLjA7XG4gIHZhciBkZXZpY2VFeHRZID0gdGhpcy5sZGV2aWNlRXh0WTtcbiAgaWYgKGRldmljZUV4dFkgIT0gMC4wKVxuICB7XG4gICAgeVdvcmxkID0gdGhpcy5sd29ybGRPcmdZICtcbiAgICAgICAgICAgICgoeSAtIHRoaXMubGRldmljZU9yZ1kpICogdGhpcy5sd29ybGRFeHRZIC8gZGV2aWNlRXh0WSk7XG4gIH1cbiAgcmV0dXJuIHlXb3JsZDtcbn1cblxuVHJhbnNmb3JtLnByb3RvdHlwZS5pbnZlcnNlVHJhbnNmb3JtUG9pbnQgPSBmdW5jdGlvbiAoaW5Qb2ludClcbntcbiAgdmFyIG91dFBvaW50ID1cbiAgICAgICAgICBuZXcgUG9pbnREKHRoaXMuaW52ZXJzZVRyYW5zZm9ybVgoaW5Qb2ludC54KSxcbiAgICAgICAgICAgICAgICAgIHRoaXMuaW52ZXJzZVRyYW5zZm9ybVkoaW5Qb2ludC55KSk7XG4gIHJldHVybiBvdXRQb2ludDtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBUcmFuc2Zvcm07XG4iLCJmdW5jdGlvbiBVbmlxdWVJREdlbmVyZXRvcigpIHtcbn1cblxuVW5pcXVlSURHZW5lcmV0b3IubGFzdElEID0gMDtcblxuVW5pcXVlSURHZW5lcmV0b3IuY3JlYXRlSUQgPSBmdW5jdGlvbiAob2JqKSB7XG4gIGlmIChVbmlxdWVJREdlbmVyZXRvci5pc1ByaW1pdGl2ZShvYmopKSB7XG4gICAgcmV0dXJuIG9iajtcbiAgfVxuICBpZiAob2JqLnVuaXF1ZUlEICE9IG51bGwpIHtcbiAgICByZXR1cm4gb2JqLnVuaXF1ZUlEO1xuICB9XG4gIG9iai51bmlxdWVJRCA9IFVuaXF1ZUlER2VuZXJldG9yLmdldFN0cmluZygpO1xuICBVbmlxdWVJREdlbmVyZXRvci5sYXN0SUQrKztcbiAgcmV0dXJuIG9iai51bmlxdWVJRDtcbn1cblxuVW5pcXVlSURHZW5lcmV0b3IuZ2V0U3RyaW5nID0gZnVuY3Rpb24gKGlkKSB7XG4gIGlmIChpZCA9PSBudWxsKVxuICAgIGlkID0gVW5pcXVlSURHZW5lcmV0b3IubGFzdElEO1xuICByZXR1cm4gXCJPYmplY3QjXCIgKyBpZCArIFwiXCI7XG59XG5cblVuaXF1ZUlER2VuZXJldG9yLmlzUHJpbWl0aXZlID0gZnVuY3Rpb24gKGFyZykge1xuICB2YXIgdHlwZSA9IHR5cGVvZiBhcmc7XG4gIHJldHVybiBhcmcgPT0gbnVsbCB8fCAodHlwZSAhPSBcIm9iamVjdFwiICYmIHR5cGUgIT0gXCJmdW5jdGlvblwiKTtcbn1cblxubW9kdWxlLmV4cG9ydHMgPSBVbmlxdWVJREdlbmVyZXRvcjtcbiIsInZhciBDb1NFRWRnZSA9IHJlcXVpcmUoJy4vQ29TRUVkZ2UnKTtcclxuXHJcbmZ1bmN0aW9uIFZpc2liaWxpdHlFZGdlKHNvdXJjZSwgdGFyZ2V0LCB2RWRnZSkge1xyXG4gIENvU0VFZGdlLmNhbGwodGhpcywgc291cmNlLCB0YXJnZXQsIHZFZGdlKTtcclxufVxyXG5cclxuVmlzaWJpbGl0eUVkZ2UucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShDb1NFRWRnZS5wcm90b3R5cGUpO1xyXG5mb3IgKHZhciBwcm9wIGluIENvU0VFZGdlKSB7XHJcbiAgVmlzaWJpbGl0eUVkZ2VbcHJvcF0gPSBDb1NFRWRnZVtwcm9wXTtcclxufVxyXG5cclxuLyoqXHJcbiogV2Ugd2FudCB0aGUgbGVuZ3RoIG9mIGEgdmlzaWJpbGl0eSBlZGdlIGNhbGN1bGF0ZWQgYXMgdGhlIGRpc3RhbmNlXHJcbiogYmV0d2VlbiBib3JkZXJzIG9mIHR3byBub2Rlcywgbm90IHRoZSBkaXN0YW5jZSBiZXR3ZWVuIGNlbnRlciBwb2ludHMuXHJcbiogRWRnZXMgaGF2ZSB0byBiZSB2ZXJ0aWNhbCBvciBob3Jpem9udGFsLlxyXG4qL1xyXG4vLyBAT3ZlcnJpZGVcclxuVmlzaWJpbGl0eUVkZ2UucHJvdG90eXBlLnVwZGF0ZUxlbmd0aCA9IGZ1bmN0aW9uICgpXHJcbntcclxuICAgIGlmICh0aGlzLnNvdXJjZS5nZXRCb3R0b20oKSA8PSB0aGlzLnRhcmdldC5nZXRUb3AoKSlcclxuICAgIHtcclxuICAgICAgICB0aGlzLmxlbmd0aFggPSAwO1xyXG4gICAgICAgIHRoaXMubGVuZ3RoWSA9IHRoaXMuc291cmNlLmdldEJvdHRvbSgpIC0gdGhpcy50YXJnZXQuZ2V0VG9wKCk7XHJcbiAgICB9IFxyXG4gICAgZWxzZSBpZiAodGhpcy5zb3VyY2UuZ2V0UmlnaHQoKSA8PSB0aGlzLnRhcmdldC5nZXRMZWZ0KCkpXHJcbiAgICB7XHJcbiAgICAgICAgdGhpcy5sZW5ndGhYID0gdGhpcy5zb3VyY2UuZ2V0UmlnaHQoKSAtIHRoaXMudGFyZ2V0LmdldExlZnQoKTtcclxuICAgICAgICB0aGlzLmxlbmd0aFkgPSAwO1xyXG4gICAgfVxyXG4gICAgLy8gZWxzZVxyXG4gICAgLy8gU3lzdGVtLm91dC5wcmludGxuKFwidW5leHBlY3RlZCBlZGdlXCIpO1xyXG5cclxuICAgIHRoaXMubGVuZ3RoID0gTWF0aC5zcXJ0KCh0aGlzLmxlbmd0aFggKiB0aGlzLmxlbmd0aFgpICsgKHRoaXMubGVuZ3RoWSAqIHRoaXMubGVuZ3RoWSkpO1xyXG59O1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBWaXNpYmlsaXR5RWRnZTtcclxuIiwidmFyIEludGVnZXIgPSByZXF1aXJlKCcuL0ludGVnZXInKTtcclxudmFyIFJlY3RhbmdsZUQgPSByZXF1aXJlKCcuL1JlY3RhbmdsZUQnKTtcclxuXHJcbnZhciBMR3JhcGggPSByZXF1aXJlKCcuL0xHcmFwaCcpO1xyXG52YXIgVmlzaWJpbGl0eUVkZ2UgPSByZXF1aXJlKCcuL1Zpc2liaWxpdHlFZGdlJyk7XHJcbnZhciBDb21wYWN0aW9uID0gcmVxdWlyZSgnLi9Db21wYWN0aW9uJyk7XHJcblxyXG5mdW5jdGlvbiBWaXNpYmlsaXR5R3JhcGgocGFyZW50LCBncmFwaE1nciwgdkdyYXBoKSBcclxue1xyXG4gICAgTEdyYXBoLmNhbGwodGhpcywgcGFyZW50LCBncmFwaE1nciwgdkdyYXBoKTtcclxuICAgIFxyXG4gICAgdGhpcy5kaXJlY3Rpb24gPSBudWxsO1xyXG59XHJcblxyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlID0gT2JqZWN0LmNyZWF0ZShMR3JhcGgucHJvdG90eXBlKTtcclxuZm9yICh2YXIgcHJvcCBpbiBMR3JhcGgpIHtcclxuICBWaXNpYmlsaXR5R3JhcGhbcHJvcF0gPSBMR3JhcGhbcHJvcF07XHJcbn1cclxuXHJcbi8qKlxyXG4qIENyZWF0ZSBhIG5ldyB2aXNpYmlsaXR5IGdyYXBoLiBDb21wYXJlIGVhY2ggdmVydGljZXMgaW4gdGhlIGdyYXBoIHRvIHNlZVxyXG4qIGlmIHRoZXkgYXJlIHZpc2libGUgdG8gZWFjaCBvdGhlci4gSWYgdHdvIHZlcnRpY2VzIGNhbiBzZWUgZWFjaCBvdGhlciBhbmRcclxuKiB0aGV5IHNlZSBlYWNoIG90aGVyIGluIHRoZSBkZXNpcmVkIGRpcmVjdGlvbiwgYWRkIGFuIGVkZ2UgYmV0d2VlbiB0aGVtLlxyXG4qL1xyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlLmNvbnN0cnVjdCA9IGZ1bmN0aW9uIChkLCB2ZXJ0aWNlcylcclxue1xyXG4gICAgdGhpcy5pbml0KHZlcnRpY2VzKTtcclxuXHJcbiAgICB2YXIgbm9kZXMgPSB0aGlzLmdldE5vZGVzKCk7XHJcbiAgICB0aGlzLmRpcmVjdGlvbiA9IGQ7XHJcblxyXG4gICAgLy8gY2hlY2sgdGhlIHZpc2liaWxpdHkgYmV0d2VlbiBlYWNoIHR3byB2ZXJ0ZXhcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbm9kZXMubGVuZ3RoOyBpKyspXHJcbiAgICB7XHJcbiAgICAgICAgZm9yICh2YXIgaiA9IGkgKyAxOyBqIDwgbm9kZXMubGVuZ3RoOyBqKyspXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICB2YXIgbm9kZTEgPSBub2Rlc1tpXTtcclxuICAgICAgICAgICAgdmFyIG5vZGUyID0gbm9kZXNbal07XHJcblxyXG4gICAgICAgICAgICB2YXIgcmVzdWx0ID0gdGhpcy5maW5kVmlzaWJpbGl0eURpcmVjdGlvbihub2RlMSwgbm9kZTIpO1xyXG5cclxuICAgICAgICAgICAgLy8gaWYgdGhleSBhcmUgdmlzaWJsZSwgY3JlYXRlIGVkZ2UuXHJcbiAgICAgICAgICAgIGlmIChyZXN1bHQgIT09IDApXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHRoaXMuY3JlYXRlRWRnZShub2RlMSwgbm9kZTIpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgYWRkcyB0aGUgZ2l2ZW4gbm9kZXMgdG8gdGhlIGdyYXBoLlxyXG4qL1xyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlLmluaXQgPSBmdW5jdGlvbiAodmVydGljZXMpXHJcbntcclxuICAgIC8vIGNyZWF0ZSB0aGUgbmV3IGdyYXBoIHdpdGggZ2l2ZW4gdmVydGljZXNcclxuICAgIHZhciBudW1PZlZlcnRpY2VzID0gdmVydGljZXMubGVuZ3RoO1xyXG4gICAgZm9yICh2YXIgaT0wOyBpPG51bU9mVmVydGljZXM7IGkrKylcclxuICAgIHtcclxuICAgICAgICB0aGlzLmFkZCh2ZXJ0aWNlc1tpXSk7XHJcbiAgICB9XHJcbn07XHJcblxyXG4vKipcclxuKiBHaXZlbiB0d28gbm9kZXMsIGNoZWNrIHRoZWlyIHZpc2liaWxpdHkuIFR3byBub2RlcyBhcmUgdmlzaWJsZSB0byBlYWNoXHJcbiogb3RoZXIgaWYgdGhlcmUgZXhpc3RzIGFuIGluZmluaXRlIHJheSB0aGF0IGludGVyc2VjdHMgdGhlbSB3aXRob3V0XHJcbiogaW50ZXJzZWN0aW5nIGFueSBvdGhlciBub2RlcyBpbiBiZXR3ZWVuIHRob3NlIHR3by5cclxuKiBcclxuKiBAcmV0dXJuIDE6IHZlcnRpY2FsLCAyOiBob3Jpem9udGFsIChpZiBhbnkgZWRnZSBmb3VuZCkuIE90aGVyd2lzZSwgcmV0dXJuXHJcbiogICAgICAgICAwXHJcbiovXHJcblZpc2liaWxpdHlHcmFwaC5wcm90b3R5cGUuZmluZFZpc2liaWxpdHlEaXJlY3Rpb24gPSBmdW5jdGlvbiAocCwgcSlcclxue1xyXG4gICAgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKVxyXG4gICAge1xyXG4gICAgICAgIC8vIGVuc3VyZSB0aGF0IHAgcG9pbnRzIHRvIHRoZSBsZWZ0bW9zdCBlbGVtZW50XHJcbiAgICAgICAgaWYgKHEuZ2V0TGVmdCgpIDwgcC5nZXRMZWZ0KCkgJiYgXHJcbiAgICAgICAgICAgIHAuZ2V0TGVmdCgpIDwgKHEuZ2V0TGVmdCgpICsgcS5nZXRXaWR0aCgpKSlcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHZhciB0ZW1wID0gcDtcclxuICAgICAgICAgICAgcCA9IHE7XHJcbiAgICAgICAgICAgIHEgPSB0ZW1wO1xyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy8gY2hlY2sgaWYgdGhlcmUgZXhpc3RzIGEgcmF5XHJcbiAgICAgICAgaWYgKHAuZ2V0TGVmdCgpIDw9IHEuZ2V0TGVmdCgpICYmIFxyXG4gICAgICAgICAgICBxLmdldExlZnQoKSA8PSAocC5nZXRMZWZ0KCkgKyBwLmdldFdpZHRoKCkpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgaWYgKHRoaXMuc3dlZXBJbnRlcnNlY3RlZEFyZWEocCwgcSkpXHJcbiAgICAgICAgICAgIHtcclxuICAgICAgICAgICAgICAgIHJldHVybiAxO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgLy8gZW5zdXJlIHRoYXQgcCBwb2ludHMgdG8gdGhlIHVwcGVyIGVsZW1lbnRcclxuICAgICAgICBpZiAocS5nZXRUb3AoKSA8IHAuZ2V0VG9wKCkgJiYgXHJcbiAgICAgICAgICAgIHAuZ2V0VG9wKCkgPCAocS5nZXRUb3AoKSArIHEuZ2V0SGVpZ2h0KCkpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgdmFyIHRlbXAgPSBwO1xyXG4gICAgICAgICAgICBwID0gcTtcclxuICAgICAgICAgICAgcSA9IHRlbXA7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICAvLyBjaGVjayBpZiB0aGVyZSBleGlzdHMgYSByYXlcclxuICAgICAgICBpZiAocC5nZXRUb3AoKSA8PSBxLmdldFRvcCgpICYmIFxyXG4gICAgICAgICAgICBxLmdldFRvcCgpIDw9IChwLmdldFRvcCgpICsgcC5nZXRIZWlnaHQoKSkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICBpZiAodGhpcy5zd2VlcEludGVyc2VjdGVkQXJlYShwLCBxKSlcclxuICAgICAgICAgICAge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIDI7XHJcbiAgICAgICAgICAgIH1cclxuICAgICAgICB9XHJcbiAgICB9XHJcbiAgICByZXR1cm4gMDtcclxufTtcclxuXHJcbi8qKlxyXG4qIFN0YXJ0aW5nIGZyb20gdGhlIGludGVyc2VjdGlvbiBhcmVhIGJldHdlZW4gcCBhbmQgcSwgd2FsayBvbiBhIGxpbmVcclxuKiBwZXJwZW5kaWN1bGFyIHRvIHRoZSBkZXNpcmVkIGRpcmVjdGlvbi4gSWYgdGhlcmUgaXMgYW4gZWRnZSB0aGF0IGRvZXMgbm90XHJcbiogaW50ZXJzZWN0IHdpdGggYW55IG90aGVyIG5vZGVzLCB0aGlzIGlzIGEgdmFsaWQgZWRnZS5cclxuKiBcclxuKiBAcmV0dXJuIHRydWUgaWYgYW4gZWRnZSBleGlzdHMuIGZhbHNlIG90aGVyd2lzZS5cclxuKi9cclxuVmlzaWJpbGl0eUdyYXBoLnByb3RvdHlwZS5zd2VlcEludGVyc2VjdGVkQXJlYSA9IGZ1bmN0aW9uIChwLCBxKVxyXG57XHJcbiAgICB2YXIgZWRnZTtcclxuICAgIHZhciBpc1ZhbGlkO1xyXG4gICAgdmFyIHN0YXJ0ID0gMDtcclxuICAgIHZhciBlbmQgPSAwO1xyXG4gICAgdmFyIHJlc3VsdDtcclxuXHJcbiAgICAvLyBmaW5kIHRoZSBzd2VlcCBsaW5lIGJvcmRlcnNcclxuICAgIGlmICh0aGlzLmRpcmVjdGlvbiA9PT0gQ29tcGFjdGlvbi5Db21wYWN0aW9uRGlyZWN0aW9uRW51bS5WRVJUSUNBTClcclxuICAgIHtcclxuICAgICAgICBzdGFydCA9IHEuZ2V0TGVmdCgpO1xyXG4gICAgICAgIGVuZCA9IE1hdGgubWluKHAuZ2V0UmlnaHQoKSwgcS5nZXRSaWdodCgpKTtcclxuICAgIH1cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgc3RhcnQgPSBxLmdldFRvcCgpO1xyXG4gICAgICAgIGVuZCA9IE1hdGgubWluKHAuZ2V0Qm90dG9tKCksIHEuZ2V0Qm90dG9tKCkpO1xyXG4gICAgfVxyXG5cclxuICAgIC8vIGlmIHRoZXkgaW50ZXJzZWN0IG9ubHkgb24gdGhlIGJvcmRlcnMsIGltbWVkaWF0ZWx5IHJldHVybiBmYWxzZS5cclxuICAgIGlmIChzdGFydCA9PT0gZW5kKVxyXG4gICAge1xyXG4gICAgICAgIHJldHVybiBmYWxzZTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBjaGVjayBmb3IgYWxsIGludGVyc2VjdGVkIGFyZWFcclxuICAgIGZvciAodmFyIHN3ZWVwUG9pbnQgPSBzdGFydDsgc3dlZXBQb2ludCA8PSBlbmQ7IHN3ZWVwUG9pbnQrKylcclxuICAgIHtcclxuICAgICAgICBpc1ZhbGlkID0gdHJ1ZTtcclxuICAgICAgICBlZGdlID0gdGhpcy50cnlDb25zdHJ1Y3RpbmdFZGdlKHAsIHEsIHN3ZWVwUG9pbnQpO1xyXG5cclxuICAgICAgICAvLyBpZiBhbiBlZGdlIGlzIGNvbnN0cnVjdGVkLCBjaGVjayBpdHMgdmFsaWRpdHlcclxuICAgICAgICBpZiAoZWRnZSAhPT0gbnVsbClcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJlc3VsdCA9IHRoaXMuY2hlY2tJbnRlcm1lZGlhdGVOb2RlcyhwLCBxLCBlZGdlLCBzd2VlcFBvaW50KTtcclxuXHJcbiAgICAgICAgICAgIGlmIChzd2VlcFBvaW50ID09PSByZXN1bHQpXHJcbiAgICAgICAgICAgICAgICBpc1ZhbGlkID0gdHJ1ZTtcclxuICAgICAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICBzd2VlcFBvaW50ID0gcmVzdWx0O1xyXG4gICAgICAgICAgICAgICAgaXNWYWxpZCA9IGZhbHNlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGlmIChpc1ZhbGlkKVxyXG4gICAgICAgICAgICByZXR1cm4gdHJ1ZTtcclxuICAgIH1cclxuXHJcbiAgICByZXR1cm4gZmFsc2U7XHJcbn07XHJcblxyXG4vKipcclxuKiBUaGlzIG1ldGhvZCB0cmllcyB0byBjb25zdHJ1Y3QgYW4gZWRnZShSZWN0YW5nbGVEIHNoYXBlKSBiZXR3ZWVuIHR3b1xyXG4qIG5vZGVzLiBUaGUgcGFyYW1ldGVyIGkgaW5kaWNhdGVzIHRoZSBzdGFydGluZyBwb2ludCBvZiB0aGUgZWRnZS4gRm9yXHJcbiogZXhhbXBsZSwgaWYgYSB2ZXJ0aWNhbCBlZGdlIHRvIGJlIGNvbnN0cnVjdGVkLCBpIGlzIHRoZSB0b3AgY29vcmRpbmF0ZS5cclxuKiBGb3IgaG9yaXpvbnRhbCwgaSBpcyB0aGUgbGVmdG1vc3QgY29vcmRpbmF0ZS5cclxuKiBcclxuKiBAcmV0dXJuIGVkZ2UuIGlmIG5vIGVkZ2UgY2FuIGJlIGNvbnN0cnVjdGVkLCByZXR1cm5zIG51bGwuXHJcbiovXHJcblZpc2liaWxpdHlHcmFwaC5wcm90b3R5cGUudHJ5Q29uc3RydWN0aW5nRWRnZSA9IGZ1bmN0aW9uIChwLCBxLCBpKVxyXG57XHJcbiAgICBpZiAodGhpcy5kaXJlY3Rpb24gPT09IENvbXBhY3Rpb24uQ29tcGFjdGlvbkRpcmVjdGlvbkVudW0uVkVSVElDQUwpXHJcbiAgICB7XHJcbiAgICAgICAgLy8gY3JlYXRlIGFuIGVkZ2UgZnJvbSB1cHBlciB0byBsb3dlciBvciByZXR1cm4gZmFsc2U6ZG9lcyBub3RcclxuICAgICAgICAvLyBleGlzdFxyXG4gICAgICAgIGlmIChwLmdldFRvcCgpIDwgcS5nZXRUb3AoKSAmJiBcclxuICAgICAgICAgICAgcC5nZXRCb3R0b20oKSA8PSBxLmdldFRvcCgpKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG5ldyBSZWN0YW5nbGVEKGksIHAuZ2V0Qm90dG9tKCksIDEsIChxLmdldFRvcCgpIC0gcC5nZXRCb3R0b20oKSkpO1xyXG4gICAgICAgIH1cclxuICAgICAgICBlbHNlIGlmIChxLmdldFRvcCgpIDwgcC5nZXRUb3AoKSAmJiBcclxuICAgICAgICAgICAgICAgICAocS5nZXRUb3AoKSArIHEuZ2V0SGVpZ2h0KCkpIDw9IHAuZ2V0VG9wKCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFJlY3RhbmdsZUQoaSwgcS5nZXRCb3R0b20oKSwgMSwgKHAuZ2V0VG9wKCkgLSBxLmdldEJvdHRvbSgpKSk7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIGVsc2VcclxuICAgICAgICB7XHJcbiAgICAgICAgICAgIHJldHVybiBudWxsO1xyXG4gICAgICAgIH1cclxuICAgIH1cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgLy8gY3JlYXRlIGFuIGVkZ2UgZnJvbSBsZWZ0bW9zdCB0byByaWdodCBvciByZXR1cm4gZmFsc2U6ZG9lc1xyXG4gICAgICAgIC8vIG5vdCBleGlzdFxyXG4gICAgICAgIGlmIChwLmdldExlZnQoKSA8IHEuZ2V0TGVmdCgpICYmIFxyXG4gICAgICAgICAgICBwLmdldFJpZ2h0KCkgPD0gcS5nZXRMZWZ0KCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFJlY3RhbmdsZUQocC5nZXRSaWdodCgpLCBpLCAocS5nZXRMZWZ0KCkgLSBwLmdldFJpZ2h0KCkpLCAxKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZSBpZiAocS5nZXRMZWZ0KCkgPCBwLmdldExlZnQoKSAmJiBcclxuICAgICAgICAgICAgICAgICBxLmdldFJpZ2h0KCkgPD0gcC5nZXRMZWZ0KCkpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICByZXR1cm4gbmV3IFJlY3RhbmdsZUQocS5nZXRSaWdodCgpLCBpLCAocC5nZXRMZWZ0KCkgLSBxLmdldFJpZ2h0KCkpLCAxKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgcmV0dXJuIG51bGw7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBudWxsO1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBtZXRob2QgY2hlY2tzIGlmIHRoZSBnaXZlbiBlZGdlIGludGVyc2VjdHMgYW55IG5vZGVzIGV4Y2VwdCB0aGVcclxuKiBzb3VyY2UgYW5kIHRhcmdldCBub2Rlcy4gSWYgYW4gaW50ZXJzZWN0aW9uIGlzIGZvdW5kLCB1cGRhdGUgdGhlIHN3ZWVwXHJcbiogcG9pbnQgdG8gdGhlIGVuZCBvZiB0aGUgaW50ZXJzZWN0ZWQgbm9kZS4gT3RoZXJ3aXNlLCBkbyBub3QgY2hhbmdlIHRoZVxyXG4qIHBvaW50LlxyXG4qL1xyXG5WaXNpYmlsaXR5R3JhcGgucHJvdG90eXBlLmNoZWNrSW50ZXJtZWRpYXRlTm9kZXMgPSBmdW5jdGlvbiAocCwgcSwgZWRnZSwgc3dlZXBQb2ludClcclxue1xyXG4gICAgZm9yICh2YXIgaiA9IDA7IGogPCB0aGlzLmdldE5vZGVzKCkubGVuZ3RoOyBqKyspXHJcbiAgICB7XHJcbiAgICAgICAgdmFyIGludGVybWVkaWF0ZU5vZGUgPSB0aGlzLmdldE5vZGVzKClbal07XHJcblxyXG4gICAgICAgIGlmIChpbnRlcm1lZGlhdGVOb2RlICE9PSBwICYmIGludGVybWVkaWF0ZU5vZGUgIT09IHEpXHJcbiAgICAgICAge1xyXG4gICAgICAgICAgICAvLyBpZiB0aGVyZSBpcyBhbiBpbnRlcnNlY3Rpb24sIGVkZ2UgaXMgbm90IHZhbGlkXHJcbiAgICAgICAgICAgIGlmIChlZGdlLmludGVyc2VjdHMoaW50ZXJtZWRpYXRlTm9kZS5nZXRSZWN0KCkpKVxyXG4gICAgICAgICAgICB7XHJcbiAgICAgICAgICAgICAgICAvLyBqdW1wIHRvIHRoZSBlbmQgb2YgaW50ZXJzZWN0ZWQgbm9kZVxyXG4gICAgICAgICAgICAgICAgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKVxyXG4gICAgICAgICAgICAgICAgICAgIHN3ZWVwUG9pbnQgPSAoaW50ZXJtZWRpYXRlTm9kZS5nZXRSaWdodCgpICsgMSk7XHJcbiAgICAgICAgICAgICAgICBlbHNlIGlmICh0aGlzLmRpcmVjdGlvbiA9PT0gQ29tcGFjdGlvbi5Db21wYWN0aW9uRGlyZWN0aW9uRW51bS5IT1JJWk9OVEFMKVxyXG4gICAgICAgICAgICAgICAgICAgIHN3ZWVwUG9pbnQgPSAoaW50ZXJtZWRpYXRlTm9kZS5nZXRCb3R0b20oKSArIDEpO1xyXG5cclxuICAgICAgICAgICAgICAgIGJyZWFrO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG5cclxuICAgIHJldHVybiBzd2VlcFBvaW50O1xyXG59O1xyXG5cclxuLyoqXHJcbiogVGhpcyBjbGFzcyBjcmVhdGVzIGFuIGVkZ2UgYmV0d2VlbiB0aGUgZ2l2ZW4gbm9kZXMgdXNpbmcgdGhlIGdpdmVuXHJcbiogZGlyZWN0aW9uLiBXaGlsZSBhZGRpbmcgdGhlIGVkZ2UsIGJlIGNhcmVmdWwgYWJvdXQgdGhlIHNvdXJjZSBhbmQgdGFyZ2V0XHJcbiogaS5lLiBpZiB3ZSB3YW50IHRvIGdldCBhIHZlcnRpY2FsIHZpc2liaWxpdHkgZ3JhcGgsIChBIC0+IEIpIEEgc2hvdWxkXHJcbiogaGF2ZSBhIGxvd2VyIHkgY29vcmRpbmF0ZSAodXBwZXIpLiBTaW1pbGFybHksIGZvciBob3Jpem9udGFsIHZpc2liaWxpdHlcclxuKiBncmFwaCAoQSAtPiBCKTogQSBpcyBvbiB0aGUgbGVmdCwgaGFzIGxvd2VyIHggY29vcmRpbmF0ZS5cclxuKi9cclxuVmlzaWJpbGl0eUdyYXBoLnByb3RvdHlwZS5jcmVhdGVFZGdlID0gZnVuY3Rpb24gKG5vZGUxLCBub2RlMilcclxue1xyXG4gICAgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLlZFUlRJQ0FMKVxyXG4gICAge1xyXG4gICAgICAgIGlmIChub2RlMS5nZXRUb3AoKSA8IG5vZGUyLmdldFRvcCgpKVxyXG4gICAgICAgICAgICB0aGlzLmFkZChuZXcgVmlzaWJpbGl0eUVkZ2Uobm9kZTEsIG5vZGUyLCBudWxsKSwgbm9kZTEsIG5vZGUyKTtcclxuICAgICAgICBlbHNlXHJcbiAgICAgICAgICAgIHRoaXMuYWRkKG5ldyBWaXNpYmlsaXR5RWRnZShub2RlMiwgbm9kZTEsIG51bGwpLCBub2RlMiwgbm9kZTEpO1xyXG4gICAgfVxyXG5cclxuICAgIGVsc2UgaWYgKHRoaXMuZGlyZWN0aW9uID09PSBDb21wYWN0aW9uLkNvbXBhY3Rpb25EaXJlY3Rpb25FbnVtLkhPUklaT05UQUwpXHJcbiAgICB7XHJcbiAgICAgICAgaWYgKG5vZGUxLmdldExlZnQoKSA8IG5vZGUyLmdldExlZnQoKSlcclxuICAgICAgICAgICAgdGhpcy5hZGQobmV3IFZpc2liaWxpdHlFZGdlKG5vZGUxLCBub2RlMiwgbnVsbCksIG5vZGUxLCBub2RlMik7XHJcbiAgICAgICAgZWxzZVxyXG4gICAgICAgICAgICB0aGlzLmFkZChuZXcgVmlzaWJpbGl0eUVkZ2Uobm9kZTIsIG5vZGUxLCBudWxsKSwgbm9kZTIsIG5vZGUxKTtcclxuICAgIH1cclxuXHJcbiAgICAvLyBjYWxjdWxhdGUgbmV3bHkgYWRkZWQgZWRnZSdzIGxlbmd0aC5cclxuICAgIHRoaXMuZ2V0RWRnZXMoKVt0aGlzLmdldEVkZ2VzKCkubGVuZ3RoIC0gMV0udXBkYXRlTGVuZ3RoKCk7XHJcbn07XHJcblxyXG4vKipcclxuKiBGb3IgZWFjaCBlZGdlIGhhdmluZyBzIGFzIGl0cyB0YXJnZXQgbm9kZSwgZmluZCBhbmQgcmV0dXJuIHRoZSBzaG9ydGVzdFxyXG4qIG9uZS4gUmV0dXJucyBudWxsIGlmIGNvdWxkIG5vdCBmaW5kIGFuIGVkZ2UuXHJcbiovXHJcblZpc2liaWxpdHlHcmFwaC5wcm90b3R5cGUuZmluZFNob3J0ZXN0RWRnZSA9IGZ1bmN0aW9uIChzKVxyXG57XHJcbiAgICB2YXIgc2hvcnRlc3RFZGdlID0gbnVsbDtcclxuICAgIHZhciBtaW5MZW5ndGggPSBJbnRlZ2VyLk1BWF9WQUxVRTtcclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMuZ2V0RWRnZXMoKS5sZW5ndGg7IGkrKylcclxuICAgIHtcclxuICAgICAgICB2YXIgZSA9IHRoaXMuZ2V0RWRnZXMoKVtpXTtcclxuXHJcbiAgICAgICAgZS51cGRhdGVMZW5ndGgoKTtcclxuICAgICAgICBpZiAoZS5nZXRUYXJnZXQoKSA9PT0gcyAmJiBlLmdldExlbmd0aCgpIDwgbWluTGVuZ3RoKVxyXG4gICAgICAgIHtcclxuICAgICAgICAgICAgc2hvcnRlc3RFZGdlID0gZTtcclxuICAgICAgICAgICAgbWluTGVuZ3RoID0gZS5nZXRMZW5ndGgoKTtcclxuICAgICAgICB9XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIHNob3J0ZXN0RWRnZTtcclxufTtcclxuXHJcbm1vZHVsZS5leHBvcnRzID0gVmlzaWJpbGl0eUdyYXBoO1xyXG4iLCIndXNlIHN0cmljdCc7XHJcblxyXG52YXIgVGhyZWFkO1xyXG5cclxudmFyIERpbWVuc2lvbkQgPSByZXF1aXJlKCcuL0RpbWVuc2lvbkQnKTtcclxudmFyIEhhc2hNYXAgPSByZXF1aXJlKCcuL0hhc2hNYXAnKTtcclxudmFyIEhhc2hTZXQgPSByZXF1aXJlKCcuL0hhc2hTZXQnKTtcclxudmFyIElHZW9tZXRyeSA9IHJlcXVpcmUoJy4vSUdlb21ldHJ5Jyk7XHJcbnZhciBJTWF0aCA9IHJlcXVpcmUoJy4vSU1hdGgnKTtcclxudmFyIEludGVnZXIgPSByZXF1aXJlKCcuL0ludGVnZXInKTtcclxudmFyIFBvaW50ID0gcmVxdWlyZSgnLi9Qb2ludCcpO1xyXG52YXIgUG9pbnREID0gcmVxdWlyZSgnLi9Qb2ludEQnKTtcclxudmFyIFJhbmRvbVNlZWQgPSByZXF1aXJlKCcuL1JhbmRvbVNlZWQnKTtcclxudmFyIFJlY3RhbmdsZUQgPSByZXF1aXJlKCcuL1JlY3RhbmdsZUQnKTtcclxudmFyIFRyYW5zZm9ybSA9IHJlcXVpcmUoJy4vVHJhbnNmb3JtJyk7XHJcbnZhciBVbmlxdWVJREdlbmVyZXRvciA9IHJlcXVpcmUoJy4vVW5pcXVlSURHZW5lcmV0b3InKTtcclxudmFyIExHcmFwaE9iamVjdCA9IHJlcXVpcmUoJy4vTEdyYXBoT2JqZWN0Jyk7XHJcbnZhciBMR3JhcGggPSByZXF1aXJlKCcuL0xHcmFwaCcpO1xyXG52YXIgTEVkZ2UgPSByZXF1aXJlKCcuL0xFZGdlJyk7XHJcbnZhciBMR3JhcGhNYW5hZ2VyID0gcmVxdWlyZSgnLi9MR3JhcGhNYW5hZ2VyJyk7XHJcbnZhciBMTm9kZSA9IHJlcXVpcmUoJy4vTE5vZGUnKTtcclxudmFyIExheW91dCA9IHJlcXVpcmUoJy4vTGF5b3V0Jyk7XHJcbnZhciBMYXlvdXRDb25zdGFudHMgPSByZXF1aXJlKCcuL0xheW91dENvbnN0YW50cycpO1xyXG52YXIgRkRMYXlvdXQgPSByZXF1aXJlKCcuL0ZETGF5b3V0Jyk7XHJcbnZhciBGRExheW91dENvbnN0YW50cyA9IHJlcXVpcmUoJy4vRkRMYXlvdXRDb25zdGFudHMnKTtcclxudmFyIEZETGF5b3V0RWRnZSA9IHJlcXVpcmUoJy4vRkRMYXlvdXRFZGdlJyk7XHJcbnZhciBGRExheW91dE5vZGUgPSByZXF1aXJlKCcuL0ZETGF5b3V0Tm9kZScpO1xyXG52YXIgQ29TRUNvbnN0YW50cyA9IHJlcXVpcmUoJy4vQ29TRUNvbnN0YW50cycpO1xyXG52YXIgQ29TRUVkZ2UgPSByZXF1aXJlKCcuL0NvU0VFZGdlJyk7XHJcbnZhciBDb1NFR3JhcGggPSByZXF1aXJlKCcuL0NvU0VHcmFwaCcpO1xyXG52YXIgQ29TRUdyYXBoTWFuYWdlciA9IHJlcXVpcmUoJy4vQ29TRUdyYXBoTWFuYWdlcicpO1xyXG52YXIgQ29TRUxheW91dCA9IHJlcXVpcmUoJy4vQ29TRUxheW91dCcpO1xyXG52YXIgQ29TRU5vZGUgPSByZXF1aXJlKCcuL0NvU0VOb2RlJyk7XHJcbnZhciBDb21wYWN0aW9uID0gcmVxdWlyZSgnLi9Db21wYWN0aW9uJyk7XHJcbnZhciBTYmduUERDb25zdGFudHMgPSByZXF1aXJlKCcuL1NiZ25QRENvbnN0YW50cycpO1xyXG52YXIgU2JnblBERWRnZSA9IHJlcXVpcmUoJy4vU2JnblBERWRnZScpO1xyXG52YXIgU2JnblBETGF5b3V0ID0gcmVxdWlyZSgnLi9TYmduUERMYXlvdXQnKTtcclxudmFyIFNiZ25QRE5vZGUgPSByZXF1aXJlKCcuL1NiZ25QRE5vZGUnKTtcclxudmFyIFNiZ25Qcm9jZXNzTm9kZSA9IHJlcXVpcmUoJy4vU2JnblByb2Nlc3NOb2RlJyk7XHJcbnZhciBWaXNpYmlsaXR5RWRnZSA9IHJlcXVpcmUoJy4vVmlzaWJpbGl0eUVkZ2UnKTtcclxudmFyIFZpc2liaWxpdHlHcmFwaCA9IHJlcXVpcmUoJy4vVmlzaWJpbGl0eUdyYXBoJyk7XHJcbnZhciBNZW1iZXJQYWNrID0gcmVxdWlyZSgnLi9NZW1iZXJQYWNrJyk7XHJcbnZhciBPcmdhbml6YXRpb24gPSByZXF1aXJlKCcuL09yZ2FuaXphdGlvbicpO1xyXG52YXIgUG9seW9taW5vUXVpY2tTb3J0ID0gcmVxdWlyZSgnLi9Qb2x5b21pbm9RdWlja1NvcnQnKTtcclxudmFyIFBvbHlvbWlub1BhY2tpbmcgPSByZXF1aXJlKCcuL1BvbHlvbWlub1BhY2tpbmcnKTtcclxudmFyIFJlY3RQcm9jID0gcmVxdWlyZSgnLi9SZWN0UHJvYycpO1xyXG5cclxuX1NiZ25QRExheW91dC5pZFRvTE5vZGUgPSB7fTtcclxuX1NiZ25QRExheW91dC50b0JlVGlsZWQgPSB7fTtcclxuXHJcbi8vIFRPRE86IEJ1bmxhciBsYXlvdXQgZGVmYXVsdCdsYXJpIG1pPyBZb2tzYSBDb1NFIHNwZWNpZmljIGRlZmF1bHQnbGFyIG1pP1xyXG5cclxudmFyIGRlZmF1bHRzID0ge1xyXG4gICAgLy8gQ2FsbGVkIG9uIGBsYXlvdXRyZWFkeWBcclxuICAgIHJlYWR5OiBmdW5jdGlvbiAoKSB7XHJcbiAgICB9LFxyXG4gICAgLy8gQ2FsbGVkIG9uIGBsYXlvdXRzdG9wYFxyXG4gICAgc3RvcDogZnVuY3Rpb24gKCkge1xyXG4gICAgfSxcclxuICAgIC8vIFdoZXRoZXIgdG8gZml0IHRoZSBuZXR3b3JrIHZpZXcgYWZ0ZXIgd2hlbiBkb25lXHJcbiAgICBmaXQ6IHRydWUsXHJcbiAgICAvLyBQYWRkaW5nIG9uIGZpdFxyXG4gICAgcGFkZGluZzogMTAsXHJcbiAgICAvLyBXaGV0aGVyIHRvIGVuYWJsZSBpbmNyZW1lbnRhbCBtb2RlXHJcbiAgICByYW5kb21pemU6IHRydWUsXHJcbiAgICAvLyBOb2RlIHJlcHVsc2lvbiAobm9uIG92ZXJsYXBwaW5nKSBtdWx0aXBsaWVyXHJcbiAgICBub2RlUmVwdWxzaW9uOiA0NTAwLFxyXG4gICAgLy8gSWRlYWwgZWRnZSAobm9uIG5lc3RlZCkgbGVuZ3RoXHJcbiAgICBpZGVhbEVkZ2VMZW5ndGg6IDUwLFxyXG4gICAgLy8gRGl2aXNvciB0byBjb21wdXRlIGVkZ2UgZm9yY2VzXHJcbiAgICBlZGdlRWxhc3RpY2l0eTogMC40NSxcclxuICAgIC8vIE5lc3RpbmcgZmFjdG9yIChtdWx0aXBsaWVyKSB0byBjb21wdXRlIGlkZWFsIGVkZ2UgbGVuZ3RoIGZvciBuZXN0ZWQgZWRnZXNcclxuICAgIG5lc3RpbmdGYWN0b3I6IDAuMSxcclxuICAgIC8vIEdyYXZpdHkgZm9yY2UgKGNvbnN0YW50KVxyXG4gICAgZ3Jhdml0eTogMC4yNSxcclxuICAgIC8vIE1heGltdW0gbnVtYmVyIG9mIGl0ZXJhdGlvbnMgdG8gcGVyZm9ybVxyXG4gICAgbnVtSXRlcjogMjUwMCxcclxuICAgIC8vIEZvciBlbmFibGluZyB0aWxpbmdcclxuICAgIHRpbGU6IHRydWUsXHJcbiAgICAvLyBUeXBlIG9mIGxheW91dCBhbmltYXRpb24uIFRoZSBvcHRpb24gc2V0IGlzIHsnZHVyaW5nJywgJ2VuZCcsIGZhbHNlfVxyXG4gICAgYW5pbWF0ZTogJ2VuZCcsXHJcbiAgICAvLyBSZXByZXNlbnRzIHRoZSBhbW91bnQgb2YgdGhlIHZlcnRpY2FsIHNwYWNlIHRvIHB1dCBiZXR3ZWVuIHRoZSB6ZXJvIGRlZ3JlZSBtZW1iZXJzIGR1cmluZyB0aGUgdGlsaW5nIG9wZXJhdGlvbihjYW4gYWxzbyBiZSBhIGZ1bmN0aW9uKVxyXG4gICAgdGlsaW5nUGFkZGluZ1ZlcnRpY2FsOiAxMCxcclxuICAgIC8vIFJlcHJlc2VudHMgdGhlIGFtb3VudCBvZiB0aGUgaG9yaXpvbnRhbCBzcGFjZSB0byBwdXQgYmV0d2VlbiB0aGUgemVybyBkZWdyZWUgbWVtYmVycyBkdXJpbmcgdGhlIHRpbGluZyBvcGVyYXRpb24oY2FuIGFsc28gYmUgYSBmdW5jdGlvbilcclxuICAgIHRpbGluZ1BhZGRpbmdIb3Jpem9udGFsOiAxMCxcclxuICAgIC8vIEdyYXZpdHkgcmFuZ2UgKGNvbnN0YW50KSBmb3IgY29tcG91bmRzXHJcbiAgICBncmF2aXR5UmFuZ2VDb21wb3VuZDogMS41LFxyXG4gICAgLy8gR3Jhdml0eSBmb3JjZSAoY29uc3RhbnQpIGZvciBjb21wb3VuZHNcclxuICAgIGdyYXZpdHlDb21wb3VuZDogMS4wLFxyXG4gICAgLy8gR3Jhdml0eSByYW5nZSAoY29uc3RhbnQpXHJcbiAgICBncmF2aXR5UmFuZ2U6IDMuOFxyXG59O1xyXG5cclxuZnVuY3Rpb24gZXh0ZW5kKGRlZmF1bHRzLCBvcHRpb25zKSB7XHJcbiAgICB2YXIgb2JqID0ge307XHJcblxyXG4gICAgZm9yICh2YXIgaSBpbiBkZWZhdWx0cykge1xyXG4gICAgICAgIG9ialtpXSA9IGRlZmF1bHRzW2ldO1xyXG4gICAgfVxyXG5cclxuICAgIGZvciAodmFyIGkgaW4gb3B0aW9ucykge1xyXG4gICAgICAgIG9ialtpXSA9IG9wdGlvbnNbaV07XHJcbiAgICB9XHJcblxyXG4gICAgcmV0dXJuIG9iajtcclxufVxyXG47XHJcblxyXG5fU2JnblBETGF5b3V0LmxheW91dCA9IG5ldyBTYmduUERMYXlvdXQoKTtcclxuZnVuY3Rpb24gX1NiZ25QRExheW91dChvcHRpb25zKSB7XHJcblxyXG4gICAgdGhpcy5vcHRpb25zID0gZXh0ZW5kKGRlZmF1bHRzLCBvcHRpb25zKTtcclxuICAgIF9TYmduUERMYXlvdXQuZ2V0VXNlck9wdGlvbnModGhpcy5vcHRpb25zKTtcclxufVxyXG5cclxuX1NiZ25QRExheW91dC5nZXRVc2VyT3B0aW9ucyA9IGZ1bmN0aW9uIChvcHRpb25zKSBcclxue1xyXG4gICAgLyoqIFRPRE86IERvIHdlIG5lZWQgbW9yZSBjb25zdGFuc3RzIChTQkdOIHNwZWNpZmljKSBoZXJlPyAqL1xyXG4gICAgXHJcbiAgICBpZiAob3B0aW9ucy5ub2RlUmVwdWxzaW9uICE9IG51bGwpXHJcbiAgICAgICAgU2JnblBEQ29uc3RhbnRzLkRFRkFVTFRfUkVQVUxTSU9OX1NUUkVOR1RIID0gQ29TRUNvbnN0YW50cy5ERUZBVUxUX1JFUFVMU0lPTl9TVFJFTkdUSCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfUkVQVUxTSU9OX1NUUkVOR1RIID0gb3B0aW9ucy5ub2RlUmVwdWxzaW9uO1xyXG4gICAgaWYgKG9wdGlvbnMuaWRlYWxFZGdlTGVuZ3RoICE9IG51bGwpXHJcbiAgICAgICAgU2JnblBEQ29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEggPSBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfRURHRV9MRU5HVEggPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX0VER0VfTEVOR1RIID0gb3B0aW9ucy5pZGVhbEVkZ2VMZW5ndGg7XHJcbiAgICBpZiAob3B0aW9ucy5lZGdlRWxhc3RpY2l0eSAhPSBudWxsKVxyXG4gICAgICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX1NQUklOR19TVFJFTkdUSCA9IENvU0VDb25zdGFudHMuREVGQVVMVF9TUFJJTkdfU1RSRU5HVEggPSBGRExheW91dENvbnN0YW50cy5ERUZBVUxUX1NQUklOR19TVFJFTkdUSCA9IG9wdGlvbnMuZWRnZUVsYXN0aWNpdHk7XHJcbiAgICBpZiAob3B0aW9ucy5uZXN0aW5nRmFjdG9yICE9IG51bGwpXHJcbiAgICAgICAgU2JnblBEQ29uc3RhbnRzLlBFUl9MRVZFTF9JREVBTF9FREdFX0xFTkdUSF9GQUNUT1IgPSBDb1NFQ29uc3RhbnRzLlBFUl9MRVZFTF9JREVBTF9FREdFX0xFTkdUSF9GQUNUT1IgPSBGRExheW91dENvbnN0YW50cy5QRVJfTEVWRUxfSURFQUxfRURHRV9MRU5HVEhfRkFDVE9SID0gb3B0aW9ucy5uZXN0aW5nRmFjdG9yO1xyXG4gICAgaWYgKG9wdGlvbnMuZ3Jhdml0eSAhPSBudWxsKVxyXG4gICAgICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX0dSQVZJVFlfU1RSRU5HVEggPSBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9TVFJFTkdUSCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9TVFJFTkdUSCA9IG9wdGlvbnMuZ3Jhdml0eTtcclxuICAgIGlmIChvcHRpb25zLm51bUl0ZXIgIT0gbnVsbClcclxuICAgICAgICBTYmduUERDb25zdGFudHMuTUFYX0lURVJBVElPTlMgPSBDb1NFQ29uc3RhbnRzLk1BWF9JVEVSQVRJT05TID0gRkRMYXlvdXRDb25zdGFudHMuTUFYX0lURVJBVElPTlMgPSBvcHRpb25zLm51bUl0ZXI7XHJcbiAgICBpZiAob3B0aW9ucy5ncmF2aXR5UmFuZ2UgIT0gbnVsbClcclxuICAgICAgICBTYmduUERDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IENvU0VDb25zdGFudHMuREVGQVVMVF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfR1JBVklUWV9SQU5HRV9GQUNUT1IgPSBvcHRpb25zLmdyYXZpdHlSYW5nZTtcclxuICAgIGlmIChvcHRpb25zLmdyYXZpdHlDb21wb3VuZCAhPSBudWxsKVxyXG4gICAgICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX0NPTVBPVU5EX0dSQVZJVFlfU1RSRU5HVEggPSBDb1NFQ29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9TVFJFTkdUSCA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9TVFJFTkdUSCA9IG9wdGlvbnMuZ3Jhdml0eUNvbXBvdW5kO1xyXG4gICAgaWYgKG9wdGlvbnMuZ3Jhdml0eVJhbmdlQ29tcG91bmQgIT0gbnVsbClcclxuICAgICAgICBTYmduUERDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IENvU0VDb25zdGFudHMuREVGQVVMVF9DT01QT1VORF9HUkFWSVRZX1JBTkdFX0ZBQ1RPUiA9IEZETGF5b3V0Q29uc3RhbnRzLkRFRkFVTFRfQ09NUE9VTkRfR1JBVklUWV9SQU5HRV9GQUNUT1IgPSBvcHRpb25zLmdyYXZpdHlSYW5nZUNvbXBvdW5kO1xyXG5cclxuICAgIFNiZ25QRENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID0gQ29TRUNvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID0gRkRMYXlvdXRDb25zdGFudHMuREVGQVVMVF9JTkNSRU1FTlRBTCA9IExheW91dENvbnN0YW50cy5ERUZBVUxUX0lOQ1JFTUVOVEFMID1cclxuICAgICAgICAgICAgIShvcHRpb25zLnJhbmRvbWl6ZSk7XHJcbiAgICBTYmduUERDb25zdGFudHMuQU5JTUFURSA9IENvU0VDb25zdGFudHMuQU5JTUFURSA9IEZETGF5b3V0Q29uc3RhbnRzLkFOSU1BVEUgPSBvcHRpb25zLmFuaW1hdGU7XHJcbn07XHJcblxyXG5fU2JnblBETGF5b3V0LnByb3RvdHlwZS5ydW4gPSBmdW5jdGlvbiAoKSB7XHJcbiAgICB2YXIgbGF5b3V0ID0gdGhpcztcclxuXHJcbiAgICBfU2JnblBETGF5b3V0LmlkVG9MTm9kZSA9IHt9O1xyXG4gICAgX1NiZ25QRExheW91dC50b0JlVGlsZWQgPSB7fTtcclxuICAgIF9TYmduUERMYXlvdXQubGF5b3V0ID0gbmV3IFNiZ25QRExheW91dCgpO1xyXG4gICAgdGhpcy5jeSA9IHRoaXMub3B0aW9ucy5jeTtcclxuICAgIHZhciBhZnRlciA9IHRoaXM7XHJcblxyXG4gICAgdGhpcy5jeS50cmlnZ2VyKCdsYXlvdXRzdGFydCcpO1xyXG5cclxuICAgIHZhciBnbSA9IF9TYmduUERMYXlvdXQubGF5b3V0Lm5ld0dyYXBoTWFuYWdlcigpO1xyXG4gICAgdGhpcy5nbSA9IGdtO1xyXG5cclxuICAgIHZhciBub2RlcyA9IHRoaXMub3B0aW9ucy5lbGVzLm5vZGVzKCk7XHJcbiAgICB2YXIgZWRnZXMgPSB0aGlzLm9wdGlvbnMuZWxlcy5lZGdlcygpO1xyXG5cclxuICAgIHRoaXMucm9vdCA9IGdtLmFkZFJvb3QoKTtcclxuXHJcbi8vICAgIGlmICghdGhpcy5vcHRpb25zLnRpbGUpIHtcclxuICAgICAgICB0aGlzLnByb2Nlc3NDaGlsZHJlbkxpc3QodGhpcy5yb290LCBfU2JnblBETGF5b3V0LmdldFRvcE1vc3ROb2Rlcyhub2RlcykpO1xyXG4vLyAgICB9IGVsc2Uge1xyXG4vLyAgICAgICAgLy8gRmluZCB6ZXJvIGRlZ3JlZSBub2RlcyBhbmQgY3JlYXRlIGEgY29tcG91bmQgZm9yIGVhY2ggbGV2ZWxcclxuLy8gICAgICAgIHZhciBtZW1iZXJHcm91cHMgPSB0aGlzLmdyb3VwWmVyb0RlZ3JlZU1lbWJlcnMoKTtcclxuLy8gICAgICAgIC8vIFRpbGUgYW5kIGNsZWFyIGNoaWxkcmVuIG9mIGVhY2ggY29tcG91bmRcclxuLy8gICAgICAgIHZhciB0aWxlZE1lbWJlclBhY2sgPSB0aGlzLmNsZWFyQ29tcG91bmRzKHRoaXMub3B0aW9ucyk7XHJcbi8vICAgICAgICAvLyBTZXBhcmF0ZWx5IHRpbGUgYW5kIGNsZWFyIHplcm8gZGVncmVlIG5vZGVzIGZvciBlYWNoIGxldmVsXHJcbi8vICAgICAgICB2YXIgdGlsZWRaZXJvRGVncmVlTm9kZXMgPSB0aGlzLmNsZWFyWmVyb0RlZ3JlZU1lbWJlcnMobWVtYmVyR3JvdXBzKTtcclxuLy8gICAgfVxyXG5cclxuXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IGVkZ2VzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgdmFyIGVkZ2UgPSBlZGdlc1tpXTtcclxuICAgICAgICB2YXIgc291cmNlTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2VkZ2UuZGF0YShcInNvdXJjZVwiKV07XHJcbiAgICAgICAgdmFyIHRhcmdldE5vZGUgPSBfU2JnblBETGF5b3V0LmlkVG9MTm9kZVtlZGdlLmRhdGEoXCJ0YXJnZXRcIildO1xyXG4gICAgICAgIHZhciBlMSA9IGdtLmFkZChfU2JnblBETGF5b3V0LmxheW91dC5uZXdFZGdlKCksIHNvdXJjZU5vZGUsIHRhcmdldE5vZGUpO1xyXG4gICAgICAgIGUxLmlkID0gZWRnZS5pZCgpO1xyXG4gICAgfVxyXG5cclxuXHJcbiAgICB2YXIgdDEgPSBsYXlvdXQudGhyZWFkO1xyXG5cclxuICAgIGlmICghdDEgfHwgdDEuc3RvcHBlZCgpKSB7IC8vIHRyeSB0byByZXVzZSB0aHJlYWRzXHJcbiAgICAgICAgdDEgPSBsYXlvdXQudGhyZWFkID0gVGhyZWFkKCk7XHJcblxyXG4gICAgICAgIHQxLnJlcXVpcmUoRGltZW5zaW9uRCwgJ0RpbWVuc2lvbkQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKEhhc2hNYXAsICdIYXNoTWFwJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShIYXNoU2V0LCAnSGFzaFNldCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoSUdlb21ldHJ5LCAnSUdlb21ldHJ5Jyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShJTWF0aCwgJ0lNYXRoJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShJbnRlZ2VyLCAnSW50ZWdlcicpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoUG9pbnQsICdQb2ludCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoUG9pbnRELCAnUG9pbnREJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShSYW5kb21TZWVkLCAnUmFuZG9tU2VlZCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoUmVjdGFuZ2xlRCwgJ1JlY3RhbmdsZUQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFRyYW5zZm9ybSwgJ1RyYW5zZm9ybScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoVW5pcXVlSURHZW5lcmV0b3IsICdVbmlxdWVJREdlbmVyZXRvcicpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoTEdyYXBoT2JqZWN0LCAnTEdyYXBoT2JqZWN0Jyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShMR3JhcGgsICdMR3JhcGgnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKExFZGdlLCAnTEVkZ2UnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKExHcmFwaE1hbmFnZXIsICdMR3JhcGhNYW5hZ2VyJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShMTm9kZSwgJ0xOb2RlJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShMYXlvdXQsICdMYXlvdXQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKExheW91dENvbnN0YW50cywgJ0xheW91dENvbnN0YW50cycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoRkRMYXlvdXQsICdGRExheW91dCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoRkRMYXlvdXRDb25zdGFudHMsICdGRExheW91dENvbnN0YW50cycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoRkRMYXlvdXRFZGdlLCAnRkRMYXlvdXRFZGdlJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShGRExheW91dE5vZGUsICdGRExheW91dE5vZGUnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKENvU0VDb25zdGFudHMsICdDb1NFQ29uc3RhbnRzJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShDb1NFRWRnZSwgJ0NvU0VFZGdlJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShDb1NFR3JhcGgsICdDb1NFR3JhcGgnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKENvU0VHcmFwaE1hbmFnZXIsICdDb1NFR3JhcGhNYW5hZ2VyJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShDb1NFTGF5b3V0LCAnQ29TRUxheW91dCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoQ29TRU5vZGUsICdDb1NFTm9kZScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoQ29tcGFjdGlvbiwgJ0NvbXBhY3Rpb24nKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFNiZ25QRENvbnN0YW50cywgJ1NiZ25QRENvbnN0YW50cycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoU2JnblBERWRnZSwgJ1NiZ25QREVkZ2UnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFNiZ25QRExheW91dCwgJ1NiZ25QRExheW91dCcpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoU2JnblBETm9kZSwgJ1NiZ25QRE5vZGUnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFNiZ25Qcm9jZXNzTm9kZSwgJ1NiZ25Qcm9jZXNzTm9kZScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoVmlzaWJpbGl0eUVkZ2UsICdWaXNpYmlsaXR5RWRnZScpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoVmlzaWJpbGl0eUdyYXBoLCAnVmlzaWJpbGl0eUdyYXBoJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShNZW1iZXJQYWNrLCAnTWVtYmVyUGFjaycpO1xyXG4gICAgICAgIHQxLnJlcXVpcmUoT3JnYW5pemF0aW9uLCAnT3JnYW5pemF0aW9uJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShQb2x5b21pbm9RdWlja1NvcnQsICdQb2x5b21pbm9RdWlja1NvcnQnKTtcclxuICAgICAgICB0MS5yZXF1aXJlKFBvbHlvbWlub1BhY2tpbmcsICdQb2x5b21pbm9QYWNraW5nJyk7XHJcbiAgICAgICAgdDEucmVxdWlyZShSZWN0UHJvYywgJ1JlY3RQcm9jJyk7XHJcbiAgICB9XHJcblxyXG4gICAgdmFyIG5vZGVzID0gdGhpcy5vcHRpb25zLmVsZXMubm9kZXMoKTtcclxuICAgIHZhciBlZGdlcyA9IHRoaXMub3B0aW9ucy5lbGVzLmVkZ2VzKCk7XHJcblxyXG4gICAgLy8gRmlyc3QgSSBuZWVkIHRvIGNyZWF0ZSB0aGUgZGF0YSBzdHJ1Y3R1cmUgdG8gcGFzcyB0byB0aGUgd29ya2VyXHJcbiAgICB2YXIgcERhdGEgPSB7XHJcbiAgICAgICAgJ25vZGVzJzogW10sXHJcbiAgICAgICAgJ2VkZ2VzJzogW11cclxuICAgIH07XHJcblxyXG4gICAgLy9NYXAgdGhlIGlkcyBvZiBub2RlcyBpbiB0aGUgbGlzdCB0byBjaGVjayBpZiBhIG5vZGUgaXMgaW4gdGhlIGxpc3QgaW4gY29uc3RhbnQgdGltZVxyXG4gICAgdmFyIG5vZGVJZE1hcCA9IHt9O1xyXG5cclxuICAgIC8vRmlsbCB0aGUgbWFwIGluIGxpbmVhciB0aW1lXHJcbiAgICBmb3IgKHZhciBpID0gMDsgaSA8IG5vZGVzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgbm9kZUlkTWFwW25vZGVzW2ldLmlkKCldID0gdHJ1ZTtcclxuICAgIH1cclxuXHJcbiAgICB2YXIgbG5vZGVzID0gZ20uZ2V0QWxsTm9kZXMoKTtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbG5vZGVzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgdmFyIGxub2RlID0gbG5vZGVzW2ldO1xyXG4gICAgICAgIHZhciBub2RlSWQgPSBsbm9kZS5pZDtcclxuICAgICAgICB2YXIgY3lOb2RlID0gdGhpcy5vcHRpb25zLmN5LmdldEVsZW1lbnRCeUlkKG5vZGVJZCk7XHJcblxyXG4gICAgICAgIHZhciBwYXJlbnRJZCA9IGN5Tm9kZS5kYXRhKCdwYXJlbnQnKTtcclxuICAgICAgICBwYXJlbnRJZCA9IG5vZGVJZE1hcFtwYXJlbnRJZF0gPyBwYXJlbnRJZCA6IHVuZGVmaW5lZDtcclxuXHJcbiAgICAgICAgdmFyIHcgPSBsbm9kZS5yZWN0LndpZHRoO1xyXG4gICAgICAgIHZhciBwb3NYID0gbG5vZGUucmVjdC54O1xyXG4gICAgICAgIHZhciBwb3NZID0gbG5vZGUucmVjdC55O1xyXG4gICAgICAgIHZhciBoID0gbG5vZGUucmVjdC5oZWlnaHQ7XHJcbiAgICAgICAgdmFyIGR1bW15X3BhcmVudF9pZCA9IG51bGw7XHJcbiAgICAgICAgXHJcbiAgICAgICAgLy8gVE9ETzogSXMgaXQgY29ycmVjdD9cclxuICAgICAgICBpZiAoY3lOb2RlLnNjcmF0Y2goJ3NiZ25QZExheW91dCcpICYmIGN5Tm9kZS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnKS5kdW1teV9wYXJlbnRfaWQpXHJcbiAgICAgICAgICAgIGR1bW15X3BhcmVudF9pZCA9IGN5Tm9kZS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnKS5kdW1teV9wYXJlbnRfaWQ7XHJcblxyXG4gICAgICAgIHBEYXRhWyAnbm9kZXMnIF0ucHVzaCh7XHJcbiAgICAgICAgICAgIGlkOiBub2RlSWQsXHJcbiAgICAgICAgICAgIHBpZDogcGFyZW50SWQsXHJcbiAgICAgICAgICAgIHg6IHBvc1gsXHJcbiAgICAgICAgICAgIHk6IHBvc1ksXHJcbiAgICAgICAgICAgIHdpZHRoOiB3LFxyXG4gICAgICAgICAgICBoZWlnaHQ6IGgsXHJcbiAgICAgICAgICAgIGR1bW15X3BhcmVudF9pZDogZHVtbXlfcGFyZW50X2lkXHJcbiAgICAgICAgfSk7XHJcblxyXG4gICAgfVxyXG5cclxuICAgIHZhciBsZWRnZXMgPSBnbS5nZXRBbGxFZGdlcygpO1xyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsZWRnZXMubGVuZ3RoOyBpKyspIHtcclxuICAgICAgICB2YXIgbGVkZ2UgPSBsZWRnZXNbaV07XHJcbiAgICAgICAgdmFyIGVkZ2VJZCA9IGxlZGdlLmlkO1xyXG4gICAgICAgIHZhciBjeUVkZ2UgPSB0aGlzLm9wdGlvbnMuY3kuZ2V0RWxlbWVudEJ5SWQoZWRnZUlkKTtcclxuICAgICAgICB2YXIgc3JjTm9kZUlkID0gY3lFZGdlLnNvdXJjZSgpLmlkKCk7XHJcbiAgICAgICAgdmFyIHRndE5vZGVJZCA9IGN5RWRnZS50YXJnZXQoKS5pZCgpO1xyXG4gICAgICAgIHBEYXRhWyAnZWRnZXMnIF0ucHVzaCh7XHJcbiAgICAgICAgICAgIGlkOiBlZGdlSWQsXHJcbiAgICAgICAgICAgIHNvdXJjZTogc3JjTm9kZUlkLFxyXG4gICAgICAgICAgICB0YXJnZXQ6IHRndE5vZGVJZFxyXG4gICAgICAgIH0pO1xyXG4gICAgfVxyXG5cclxuICAgIHZhciByZWFkeSA9IGZhbHNlO1xyXG5cclxuICAgIHQxLnBhc3MocERhdGEpLnJ1bihmdW5jdGlvbiAocERhdGEpIHtcclxuICAgICAgICB2YXIgbG9nID0gZnVuY3Rpb24gKG1zZykge1xyXG4gICAgICAgICAgICBicm9hZGNhc3Qoe2xvZzogbXNnfSk7XHJcbiAgICAgICAgfTtcclxuXHJcbiAgICAgICAgbG9nKFwic3RhcnQgdGhyZWFkXCIpO1xyXG5cclxuICAgICAgICAvL3RoZSBsYXlvdXQgd2lsbCBiZSBydW4gaW4gdGhlIHRocmVhZCBhbmQgdGhlIHJlc3VsdHMgYXJlIHRvIGJlIHBhc3NlZFxyXG4gICAgICAgIC8vdG8gdGhlIG1haW4gdGhyZWFkIHdpdGggdGhlIHJlc3VsdCBtYXBcclxuICAgICAgICB2YXIgbGF5b3V0X3QgPSBuZXcgU2JnblBETGF5b3V0KCk7XHJcbiAgICAgICAgdmFyIGdtX3QgPSBsYXlvdXRfdC5uZXdHcmFwaE1hbmFnZXIoKTtcclxuICAgICAgICB2YXIgbmdyYXBoID0gZ21fdC5sYXlvdXQubmV3R3JhcGgoKTtcclxuICAgICAgICB2YXIgbm5vZGUgPSBnbV90LmxheW91dC5uZXdOb2RlKG51bGwpO1xyXG4gICAgICAgIHZhciByb290ID0gZ21fdC5hZGQobmdyYXBoLCBubm9kZSk7XHJcbiAgICAgICAgcm9vdC5ncmFwaE1hbmFnZXIgPSBnbV90O1xyXG4gICAgICAgIGdtX3Quc2V0Um9vdEdyYXBoKHJvb3QpO1xyXG4gICAgICAgIHZhciByb290X3QgPSBnbV90LnJvb3RHcmFwaDtcclxuXHJcbiAgICAgICAgLy9tYXBzIGZvciBpbm5lciB1c2FnZSBvZiB0aGUgdGhyZWFkXHJcbiAgICAgICAgdmFyIG9ycGhhbnNfdCA9IFtdO1xyXG4gICAgICAgIHZhciBpZFRvTE5vZGVfdCA9IHt9O1xyXG4gICAgICAgIHZhciBjaGlsZHJlbk1hcCA9IHt9O1xyXG5cclxuICAgICAgICAvL0EgbWFwIG9mIG5vZGUgaWQgdG8gY29ycmVzcG9uZGluZyBub2RlIHBvc2l0aW9uIGFuZCBzaXplc1xyXG4gICAgICAgIC8vaXQgaXMgdG8gYmUgcmV0dXJuZWQgYXQgdGhlIGVuZCBvZiB0aGUgdGhyZWFkIGZ1bmN0aW9uXHJcbiAgICAgICAgdmFyIHJlc3VsdCA9IHt9O1xyXG5cclxuICAgICAgICAvL3RoaXMgZnVuY3Rpb24gaXMgc2ltaWxhciB0byBwcm9jZXNzQ2hpbGRyZW5MaXN0IGZ1bmN0aW9uIGluIHRoZSBtYWluIHRocmVhZFxyXG4gICAgICAgIC8vaXQgaXMgdG8gcHJvY2VzcyB0aGUgbm9kZXMgaW4gY29ycmVjdCBvcmRlciByZWN1cnNpdmVseVxyXG4gICAgICAgIHZhciBwcm9jZXNzTm9kZXMgPSBmdW5jdGlvbiAocGFyZW50LCBjaGlsZHJlbikge1xyXG4gICAgICAgICAgICB2YXIgc2l6ZSA9IGNoaWxkcmVuLmxlbmd0aDtcclxuICAgICAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCBzaXplOyBpKyspIHtcclxuICAgICAgICAgICAgICAgIHZhciB0aGVDaGlsZCA9IGNoaWxkcmVuW2ldO1xyXG4gICAgICAgICAgICAgICAgdmFyIGNoaWxkcmVuX29mX2NoaWxkcmVuID0gY2hpbGRyZW5NYXBbdGhlQ2hpbGQuaWRdO1xyXG4gICAgICAgICAgICAgICAgdmFyIHRoZU5vZGU7XHJcblxyXG4gICAgICAgICAgICAgICAgaWYgKHRoZUNoaWxkLndpZHRoICE9IG51bGxcclxuICAgICAgICAgICAgICAgICAgICAgICAgJiYgdGhlQ2hpbGQuaGVpZ2h0ICE9IG51bGwpIHtcclxuICAgICAgICAgICAgICAgICAgICB0aGVOb2RlID0gcGFyZW50LmFkZChuZXcgU2JnblBETm9kZShnbV90LFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgbmV3IFBvaW50RCh0aGVDaGlsZC54LCB0aGVDaGlsZC55KSxcclxuICAgICAgICAgICAgICAgICAgICAgICAgICAgIG5ldyBEaW1lbnNpb25EKHBhcnNlRmxvYXQodGhlQ2hpbGQud2lkdGgpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICBwYXJzZUZsb2F0KHRoZUNoaWxkLmhlaWdodCkpKSk7XHJcbiAgICAgICAgICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICAgICAgICAgIHRoZU5vZGUgPSBwYXJlbnQuYWRkKG5ldyBTYmduUEROb2RlKGdtX3QpKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIHRoZU5vZGUuaWQgPSB0aGVDaGlsZC5pZDtcclxuICAgICAgICAgICAgICAgIGlkVG9MTm9kZV90W3RoZUNoaWxkLmlkXSA9IHRoZU5vZGU7XHJcblxyXG4gICAgICAgICAgICAgICAgaWYgKGlzTmFOKHRoZU5vZGUucmVjdC54KSkge1xyXG4gICAgICAgICAgICAgICAgICAgIHRoZU5vZGUucmVjdC54ID0gMDtcclxuICAgICAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgICAgICBpZiAoaXNOYU4odGhlTm9kZS5yZWN0LnkpKSB7XHJcbiAgICAgICAgICAgICAgICAgICAgdGhlTm9kZS5yZWN0LnkgPSAwO1xyXG4gICAgICAgICAgICAgICAgfVxyXG5cclxuICAgICAgICAgICAgICAgIGlmIChjaGlsZHJlbl9vZl9jaGlsZHJlbiAhPSBudWxsICYmIGNoaWxkcmVuX29mX2NoaWxkcmVuLmxlbmd0aCA+IDApIHtcclxuICAgICAgICAgICAgICAgICAgICB2YXIgdGhlTmV3R3JhcGg7XHJcbiAgICAgICAgICAgICAgICAgICAgdGhlTmV3R3JhcGggPSBsYXlvdXRfdC5nZXRHcmFwaE1hbmFnZXIoKS5hZGQobGF5b3V0X3QubmV3R3JhcGgoKSwgdGhlTm9kZSk7XHJcbiAgICAgICAgICAgICAgICAgICAgdGhlTmV3R3JhcGguZ3JhcGhNYW5hZ2VyID0gZ21fdDtcclxuICAgICAgICAgICAgICAgICAgICBwcm9jZXNzTm9kZXModGhlTmV3R3JhcGgsIGNoaWxkcmVuX29mX2NoaWxkcmVuKTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgfVxyXG4gICAgICAgIH1cclxuXHJcbiAgICAgICAgLy9maWxsIHRoZSBjaGlkcmVuTWFwIGFuZCBvcnBoYW5zX3QgbWFwcyB0byBwcm9jZXNzIHRoZSBub2RlcyBpbiB0aGUgY29ycmVjdCBvcmRlclxyXG4gICAgICAgIHZhciBub2RlcyA9IHBEYXRhLm5vZGVzO1xyXG4gICAgICAgIGZvciAodmFyIGkgPSAwOyBpIDwgbm9kZXMubGVuZ3RoOyBpKyspIHtcclxuICAgICAgICAgICAgdmFyIHRoZU5vZGUgPSBub2Rlc1tpXTtcclxuICAgICAgICAgICAgdmFyIHBfaWQgPSB0aGVOb2RlLnBpZDtcclxuICAgICAgICAgICAgaWYgKHBfaWQgIT0gbnVsbCkge1xyXG4gICAgICAgICAgICAgICAgaWYgKGNoaWxkcmVuTWFwW3BfaWRdID09IG51bGwpIHtcclxuICAgICAgICAgICAgICAgICAgICBjaGlsZHJlbk1hcFtwX2lkXSA9IFtdO1xyXG4gICAgICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICAgICAgY2hpbGRyZW5NYXBbcF9pZF0ucHVzaCh0aGVOb2RlKTtcclxuICAgICAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgICAgICAgIG9ycGhhbnNfdC5wdXNoKHRoZU5vZGUpO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBwcm9jZXNzTm9kZXMocm9vdF90LCBvcnBoYW5zX3QpO1xyXG5cclxuICAgICAgICAvL2hhbmRsZSB0aGUgZWRnZXNcclxuICAgICAgICB2YXIgZWRnZXMgPSBwRGF0YS5lZGdlcztcclxuICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IGVkZ2VzLmxlbmd0aDsgaSsrKSB7XHJcbiAgICAgICAgICAgIHZhciBlZGdlID0gZWRnZXNbaV07XHJcbiAgICAgICAgICAgIHZhciBzb3VyY2VOb2RlID0gaWRUb0xOb2RlX3RbZWRnZS5zb3VyY2VdO1xyXG4gICAgICAgICAgICB2YXIgdGFyZ2V0Tm9kZSA9IGlkVG9MTm9kZV90W2VkZ2UudGFyZ2V0XTtcclxuICAgICAgICAgICAgdmFyIGUxID0gZ21fdC5hZGQobGF5b3V0X3QubmV3RWRnZSgpLCBzb3VyY2VOb2RlLCB0YXJnZXROb2RlKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIC8vcnVuIHRoZSBsYXlvdXQgY3JhdGVkIGluIHRoaXMgdGhyZWFkXHJcbiAgICAgICAgbGF5b3V0X3QucnVuTGF5b3V0KCk7XHJcblxyXG4gICAgICAgIC8vZmlsbCB0aGUgcmVzdWx0IG1hcFxyXG4gICAgICAgIGZvciAodmFyIGlkIGluIGlkVG9MTm9kZV90KSB7XHJcbiAgICAgICAgICAgIHZhciBsTm9kZSA9IGlkVG9MTm9kZV90W2lkXTtcclxuICAgICAgICAgICAgdmFyIHJlY3QgPSBsTm9kZS5yZWN0O1xyXG4gICAgICAgICAgICByZXN1bHRbaWRdID0ge1xyXG4gICAgICAgICAgICAgICAgaWQ6IGlkLFxyXG4gICAgICAgICAgICAgICAgeDogcmVjdC54LFxyXG4gICAgICAgICAgICAgICAgeTogcmVjdC55LFxyXG4gICAgICAgICAgICAgICAgdzogcmVjdC53aWR0aCxcclxuICAgICAgICAgICAgICAgIGg6IHJlY3QuaGVpZ2h0XHJcbiAgICAgICAgICAgIH07XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHZhciBzZWVkcyA9IHt9O1xyXG4gICAgICAgIHNlZWRzLnJzU2VlZCA9IFJhbmRvbVNlZWQuc2VlZDtcclxuICAgICAgICBzZWVkcy5yc1ggPSBSYW5kb21TZWVkLng7XHJcbiAgICAgICAgdmFyIHBhc3MgPSB7XHJcbiAgICAgICAgICAgIHJlc3VsdDogcmVzdWx0LFxyXG4gICAgICAgICAgICBzZWVkczogc2VlZHNcclxuICAgICAgICB9XHJcbiAgICAgICAgLy9yZXR1cm4gdGhlIHJlc3VsdCBtYXAgdG8gcGFzcyBpdCB0byB0aGUgdGhlbiBmdW5jdGlvbiBhcyBwYXJhbWV0ZXJcclxuICAgICAgICByZXR1cm4gcGFzcztcclxuICAgIH0pLnRoZW4oZnVuY3Rpb24gKHBhc3MpIHtcclxuICAgICAgICB2YXIgcmVzdWx0ID0gcGFzcy5yZXN1bHQ7XHJcbiAgICAgICAgdmFyIHNlZWRzID0gcGFzcy5zZWVkcztcclxuICAgICAgICBSYW5kb21TZWVkLnNlZWQgPSBzZWVkcy5yc1NlZWQ7XHJcbiAgICAgICAgUmFuZG9tU2VlZC54ID0gc2VlZHMucnNYO1xyXG4gICAgICAgIC8vcmVmcmVzaCB0aGUgbG5vZGUgcG9zaXRpb25zIGFuZCBzaXplcyBieSB1c2luZyByZXN1bHQgbWFwXHJcbiAgICAgICAgZm9yICh2YXIgaWQgaW4gcmVzdWx0KSB7XHJcbiAgICAgICAgICAgIHZhciBsTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2lkXTtcclxuICAgICAgICAgICAgdmFyIG5vZGUgPSByZXN1bHRbaWRdO1xyXG4gICAgICAgICAgICBsTm9kZS5yZWN0LnggPSBub2RlLng7XHJcbiAgICAgICAgICAgIGxOb2RlLnJlY3QueSA9IG5vZGUueTtcclxuICAgICAgICAgICAgbE5vZGUucmVjdC53aWR0aCA9IG5vZGUudztcclxuICAgICAgICAgICAgbE5vZGUucmVjdC5oZWlnaHQgPSBub2RlLmg7XHJcbiAgICAgICAgfVxyXG4gICAgICAgIC8qIFRPRE86XHJcbiAgICAgICAgICogaWYgKGFmdGVyLm9wdGlvbnMudGlsZSkge1xyXG4gICAgICAgICAgICAvLyBSZXBvcHVsYXRlIG1lbWJlcnNcclxuICAgICAgICAgICAgLy8gVE9ETzpcclxuICAgICAgICAgICAgLy9hZnRlci5yZXBvcHVsYXRlWmVyb0RlZ3JlZU1lbWJlcnModGlsZWRaZXJvRGVncmVlTm9kZXMpO1xyXG4gICAgICAgICAgICAvL2FmdGVyLnJlcG9wdWxhdGVDb21wb3VuZHModGlsZWRNZW1iZXJQYWNrKTtcclxuICAgICAgICAgICAgYWZ0ZXIub3B0aW9ucy5lbGVzLm5vZGVzKCkudXBkYXRlQ29tcG91bmRCb3VuZHMoKTtcclxuICAgICAgICB9Ki9cclxuXHJcbiAgICAgICAgdmFyIGdldFBvc2l0aW9ucyA9IGZ1bmN0aW9uIChpLCBlbGUpIHtcclxuICAgICAgICAgICAgdmFyIHRoZUlkID0gZWxlLmRhdGEoJ2lkJyk7XHJcbiAgICAgICAgICAgIHZhciBsTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW3RoZUlkXTtcclxuXHJcbiAgICAgICAgICAgIHJldHVybiB7XHJcbiAgICAgICAgICAgICAgICB4OiBsTm9kZS5nZXRSZWN0KCkuZ2V0Q2VudGVyWCgpLFxyXG4gICAgICAgICAgICAgICAgeTogbE5vZGUuZ2V0UmVjdCgpLmdldENlbnRlclkoKVxyXG4gICAgICAgICAgICB9O1xyXG4gICAgICAgIH07XHJcblxyXG4gICAgICAgIGlmIChhZnRlci5vcHRpb25zLmFuaW1hdGUgIT09ICdkdXJpbmcnKSB7XHJcbiAgICAgICAgICAgIGFmdGVyLm9wdGlvbnMuZWxlcy5ub2RlcygpLmxheW91dFBvc2l0aW9ucyhhZnRlciwgYWZ0ZXIub3B0aW9ucywgZ2V0UG9zaXRpb25zKTtcclxuICAgICAgICB9IGVsc2Uge1xyXG4gICAgICAgICAgICBhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKS5wb3NpdGlvbnMoZ2V0UG9zaXRpb25zKTtcclxuXHJcbiAgICAgICAgICAgIGlmIChhZnRlci5vcHRpb25zLmZpdClcclxuICAgICAgICAgICAgICAgIGFmdGVyLm9wdGlvbnMuY3kuZml0KGFmdGVyLm9wdGlvbnMuZWxlcy5ub2RlcygpLCBhZnRlci5vcHRpb25zLnBhZGRpbmcpO1xyXG5cclxuICAgICAgICAgICAgLy90cmlnZ2VyIGxheW91dHJlYWR5IHdoZW4gZWFjaCBub2RlIGhhcyBoYWQgaXRzIHBvc2l0aW9uIHNldCBhdCBsZWFzdCBvbmNlXHJcbiAgICAgICAgICAgIGlmICghcmVhZHkpIHtcclxuICAgICAgICAgICAgICAgIGFmdGVyLmN5Lm9uZSgnbGF5b3V0cmVhZHknLCBhZnRlci5vcHRpb25zLnJlYWR5KTtcclxuICAgICAgICAgICAgICAgIGFmdGVyLmN5LnRyaWdnZXIoJ2xheW91dHJlYWR5Jyk7XHJcbiAgICAgICAgICAgIH1cclxuXHJcbiAgICAgICAgICAgIC8vIHRyaWdnZXIgbGF5b3V0c3RvcCB3aGVuIHRoZSBsYXlvdXQgc3RvcHMgKGUuZy4gZmluaXNoZXMpXHJcbiAgICAgICAgICAgIGFmdGVyLmN5Lm9uZSgnbGF5b3V0c3RvcCcsIGFmdGVyLm9wdGlvbnMuc3RvcCk7XHJcbiAgICAgICAgICAgIGFmdGVyLmN5LnRyaWdnZXIoJ2xheW91dHN0b3AnKTtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIHQxLnN0b3AoKTtcclxuICAgICAgICBhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKS5yZW1vdmVTY3JhdGNoKCdzYmduUGRMYXlvdXQnKTtcclxuICAgIH0pO1xyXG5cclxuICAgIHQxLm9uKCdtZXNzYWdlJywgZnVuY3Rpb24gKGUpIHtcclxuICAgICAgICB2YXIgbG9nTXNnID0gZS5tZXNzYWdlLmxvZztcclxuICAgICAgICBpZiAobG9nTXNnICE9IG51bGwpIHtcclxuICAgICAgICAgICAgY29uc29sZS5sb2coJ1RocmVhZCBsb2c6ICcgKyBsb2dNc2cpO1xyXG4gICAgICAgICAgICByZXR1cm47XHJcbiAgICAgICAgfVxyXG4gICAgICAgIHZhciBwRGF0YSA9IGUubWVzc2FnZS5wRGF0YTtcclxuICAgICAgICBpZiAocERhdGEgIT0gbnVsbCkge1xyXG4gICAgICAgICAgICBhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKS5wb3NpdGlvbnMoZnVuY3Rpb24gKGksIGVsZSkge1xyXG4gICAgICAgICAgICAgICAgaWYgKGVsZS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnKSAmJiBlbGUuc2NyYXRjaCgnc2JnblBkTGF5b3V0JykuZHVtbXlfcGFyZW50X2lkKSB7XHJcbiAgICAgICAgICAgICAgICAgICAgdmFyIGR1bW15UGFyZW50ID0gZWxlLnNjcmF0Y2goJ3NiZ25QZExheW91dCcpLmR1bW15X3BhcmVudF9pZDtcclxuICAgICAgICAgICAgICAgICAgICByZXR1cm4ge1xyXG4gICAgICAgICAgICAgICAgICAgICAgICB4OiBkdW1teVBhcmVudC54LFxyXG4gICAgICAgICAgICAgICAgICAgICAgICB5OiBkdW1teVBhcmVudC55XHJcbiAgICAgICAgICAgICAgICAgICAgfTtcclxuICAgICAgICAgICAgICAgIH1cclxuICAgICAgICAgICAgICAgIHZhciB0aGVJZCA9IGVsZS5kYXRhKCdpZCcpO1xyXG4gICAgICAgICAgICAgICAgdmFyIHBOb2RlID0gcERhdGFbdGhlSWRdO1xyXG4gICAgICAgICAgICAgICAgdmFyIHRlbXAgPSB0aGlzO1xyXG4gICAgICAgICAgICAgICAgd2hpbGUgKHBOb2RlID09IG51bGwpIHtcclxuICAgICAgICAgICAgICAgICAgICB0ZW1wID0gdGVtcC5wYXJlbnQoKVswXTtcclxuICAgICAgICAgICAgICAgICAgICBwTm9kZSA9IHBEYXRhW3RlbXAuaWQoKV07XHJcbiAgICAgICAgICAgICAgICAgICAgcERhdGFbdGhlSWRdID0gcE5vZGU7XHJcbiAgICAgICAgICAgICAgICB9XHJcbiAgICAgICAgICAgICAgICByZXR1cm4ge1xyXG4gICAgICAgICAgICAgICAgICAgIHg6IHBOb2RlLngsXHJcbiAgICAgICAgICAgICAgICAgICAgeTogcE5vZGUueVxyXG4gICAgICAgICAgICAgICAgfTtcclxuICAgICAgICAgICAgfSk7XHJcblxyXG4gICAgICAgICAgICBpZiAoYWZ0ZXIub3B0aW9ucy5maXQpXHJcbiAgICAgICAgICAgICAgICBhZnRlci5vcHRpb25zLmN5LmZpdChhZnRlci5vcHRpb25zLmVsZXMubm9kZXMoKSwgYWZ0ZXIub3B0aW9ucy5wYWRkaW5nKTtcclxuXHJcbiAgICAgICAgICAgIGlmICghcmVhZHkpIHtcclxuICAgICAgICAgICAgICAgIHJlYWR5ID0gdHJ1ZTtcclxuICAgICAgICAgICAgICAgIGFmdGVyLm9uZSgnbGF5b3V0cmVhZHknLCBhZnRlci5vcHRpb25zLnJlYWR5KTtcclxuICAgICAgICAgICAgICAgIGFmdGVyLnRyaWdnZXIoe3R5cGU6ICdsYXlvdXRyZWFkeScsIGxheW91dDogYWZ0ZXJ9KTtcclxuICAgICAgICAgICAgfVxyXG4gICAgICAgICAgICByZXR1cm47XHJcbiAgICAgICAgfVxyXG4gICAgfSk7XHJcblxyXG4gICAgcmV0dXJuIHRoaXM7IC8vIGNoYWluaW5nXHJcbn07XHJcblxyXG4vL0dldCB0aGUgdG9wIG1vc3Qgb25lcyBvZiBhIGxpc3Qgb2Ygbm9kZXNcclxuX1NiZ25QRExheW91dC5nZXRUb3BNb3N0Tm9kZXMgPSBmdW5jdGlvbiAobm9kZXMpIHtcclxuICAgIHZhciBub2Rlc01hcCA9IHt9O1xyXG4gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKykge1xyXG4gICAgICAgIG5vZGVzTWFwW25vZGVzW2ldLmlkKCldID0gdHJ1ZTtcclxuICAgIH1cclxuICAgIHZhciByb290cyA9IG5vZGVzLmZpbHRlcihmdW5jdGlvbiAoaSwgZWxlKSB7XHJcbiAgICAgICAgdmFyIHBhcmVudCA9IGVsZS5wYXJlbnQoKVswXTtcclxuICAgICAgICB3aGlsZSAocGFyZW50ICE9IG51bGwpIHtcclxuICAgICAgICAgICAgaWYgKG5vZGVzTWFwW3BhcmVudC5pZCgpXSkge1xyXG4gICAgICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4gICAgICAgICAgICB9XHJcbiAgICAgICAgICAgIHBhcmVudCA9IHBhcmVudC5wYXJlbnQoKVswXTtcclxuICAgICAgICB9XHJcbiAgICAgICAgcmV0dXJuIHRydWU7XHJcbiAgICB9KTtcclxuXHJcbiAgICByZXR1cm4gcm9vdHM7XHJcbn07XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmdldFRvQmVUaWxlZCA9IGZ1bmN0aW9uIChub2RlKSB7XHJcbi8vICAgIHZhciBpZCA9IG5vZGUuZGF0YShcImlkXCIpO1xyXG4vLyAgICAvL2ZpcnN0bHkgY2hlY2sgdGhlIHByZXZpb3VzIHJlc3VsdHNcclxuLy8gICAgaWYgKF9TYmduUERMYXlvdXQudG9CZVRpbGVkW2lkXSAhPSBudWxsKSB7XHJcbi8vICAgICAgICByZXR1cm4gX1NiZ25QRExheW91dC50b0JlVGlsZWRbaWRdO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIC8vb25seSBjb21wb3VuZCBub2RlcyBhcmUgdG8gYmUgdGlsZWRcclxuLy8gICAgdmFyIGNoaWxkcmVuID0gbm9kZS5jaGlsZHJlbigpO1xyXG4vLyAgICBpZiAoY2hpbGRyZW4gPT0gbnVsbCB8fCBjaGlsZHJlbi5sZW5ndGggPT0gMCkge1xyXG4vLyAgICAgICAgX1NiZ25QRExheW91dC50b0JlVGlsZWRbaWRdID0gZmFsc2U7XHJcbi8vICAgICAgICByZXR1cm4gZmFsc2U7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy9hIGNvbXBvdW5kIG5vZGUgaXMgbm90IHRvIGJlIHRpbGVkIGlmIGFsbCBvZiBpdHMgY29tcG91bmQgY2hpbGRyZW4gYXJlIG5vdCB0byBiZSB0aWxlZFxyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IGNoaWxkcmVuLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICB2YXIgdGhlQ2hpbGQgPSBjaGlsZHJlbltpXTtcclxuLy9cclxuLy8gICAgICAgIGlmICh0aGlzLmdldE5vZGVEZWdyZWUodGhlQ2hpbGQpID4gMCkge1xyXG4vLyAgICAgICAgICAgIF9TYmduUERMYXlvdXQudG9CZVRpbGVkW2lkXSA9IGZhbHNlO1xyXG4vLyAgICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuLy8gICAgICAgIH1cclxuLy9cclxuLy8gICAgICAgIC8vcGFzcyB0aGUgY2hpbGRyZW4gbm90IGhhdmluZyB0aGUgY29tcG91bmQgc3RydWN0dXJlXHJcbi8vICAgICAgICBpZiAodGhlQ2hpbGQuY2hpbGRyZW4oKSA9PSBudWxsIHx8IHRoZUNoaWxkLmNoaWxkcmVuKCkubGVuZ3RoID09IDApIHtcclxuLy8gICAgICAgICAgICBfU2JnblBETGF5b3V0LnRvQmVUaWxlZFt0aGVDaGlsZC5kYXRhKFwiaWRcIildID0gZmFsc2U7XHJcbi8vICAgICAgICAgICAgY29udGludWU7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICBpZiAoIXRoaXMuZ2V0VG9CZVRpbGVkKHRoZUNoaWxkKSkge1xyXG4vLyAgICAgICAgICAgIF9TYmduUERMYXlvdXQudG9CZVRpbGVkW2lkXSA9IGZhbHNlO1xyXG4vLyAgICAgICAgICAgIHJldHVybiBmYWxzZTtcclxuLy8gICAgICAgIH1cclxuLy8gICAgfVxyXG4vLyAgICBfU2JnblBETGF5b3V0LnRvQmVUaWxlZFtpZF0gPSB0cnVlO1xyXG4vLyAgICByZXR1cm4gdHJ1ZTtcclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5nZXROb2RlRGVncmVlID0gZnVuY3Rpb24gKG5vZGUpIHtcclxuLy8gICAgdmFyIGlkID0gbm9kZS5pZCgpO1xyXG4vLyAgICB2YXIgZWRnZXMgPSB0aGlzLm9wdGlvbnMuZWxlcy5lZGdlcygpLmZpbHRlcihmdW5jdGlvbiAoaSwgZWxlKSB7XHJcbi8vICAgICAgICB2YXIgc291cmNlID0gZWxlLmRhdGEoJ3NvdXJjZScpO1xyXG4vLyAgICAgICAgdmFyIHRhcmdldCA9IGVsZS5kYXRhKCd0YXJnZXQnKTtcclxuLy8gICAgICAgIGlmIChzb3VyY2UgIT0gdGFyZ2V0ICYmIChzb3VyY2UgPT0gaWQgfHwgdGFyZ2V0ID09IGlkKSkge1xyXG4vLyAgICAgICAgICAgIHJldHVybiB0cnVlO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9KTtcclxuLy8gICAgcmV0dXJuIGVkZ2VzLmxlbmd0aDtcclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5nZXROb2RlRGVncmVlV2l0aENoaWxkcmVuID0gZnVuY3Rpb24gKG5vZGUpIHtcclxuLy8gICAgdmFyIGRlZ3JlZSA9IHRoaXMuZ2V0Tm9kZURlZ3JlZShub2RlKTtcclxuLy8gICAgdmFyIGNoaWxkcmVuID0gbm9kZS5jaGlsZHJlbigpO1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IGNoaWxkcmVuLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICB2YXIgY2hpbGQgPSBjaGlsZHJlbltpXTtcclxuLy8gICAgICAgIGRlZ3JlZSArPSB0aGlzLmdldE5vZGVEZWdyZWVXaXRoQ2hpbGRyZW4oY2hpbGQpO1xyXG4vLyAgICB9XHJcbi8vICAgIHJldHVybiBkZWdyZWU7XHJcbi8vfTtcclxuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuZ3JvdXBaZXJvRGVncmVlTWVtYmVycyA9IGZ1bmN0aW9uICgpIHtcclxuLy8gICAgLy8gYXJyYXkgb2YgW3BhcmVudF9pZCB4IG9uZURlZ3JlZU5vZGVfaWRdIFxyXG4vLyAgICB2YXIgdGVtcE1lbWJlckdyb3VwcyA9IFtdO1xyXG4vLyAgICB2YXIgbWVtYmVyR3JvdXBzID0gW107XHJcbi8vICAgIHZhciBzZWxmID0gdGhpcztcclxuLy8gICAgdmFyIHBhcmVudE1hcCA9IHt9O1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRoaXMub3B0aW9ucy5lbGVzLm5vZGVzKCkubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgIHBhcmVudE1hcFt0aGlzLm9wdGlvbnMuZWxlcy5ub2RlcygpW2ldLmlkKCldID0gdHJ1ZTtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICAvLyBGaW5kIGFsbCB6ZXJvIGRlZ3JlZSBub2RlcyB3aGljaCBhcmVuJ3QgY292ZXJlZCBieSBhIGNvbXBvdW5kXHJcbi8vICAgIHZhciB6ZXJvRGVncmVlID0gdGhpcy5vcHRpb25zLmVsZXMubm9kZXMoKS5maWx0ZXIoZnVuY3Rpb24gKGksIGVsZSkge1xyXG4vLyAgICAgICAgdmFyIHBpZCA9IGVsZS5kYXRhKCdwYXJlbnQnKTtcclxuLy8gICAgICAgIGlmIChwaWQgIT0gdW5kZWZpbmVkICYmICFwYXJlbnRNYXBbcGlkXSkge1xyXG4vLyAgICAgICAgICAgIHBpZCA9IHVuZGVmaW5lZDtcclxuLy8gICAgICAgIH1cclxuLy9cclxuLy8gICAgICAgIGlmIChzZWxmLmdldE5vZGVEZWdyZWVXaXRoQ2hpbGRyZW4oZWxlKSA9PSAwICYmIChwaWQgPT0gdW5kZWZpbmVkIHx8IChwaWQgIT0gdW5kZWZpbmVkICYmICFzZWxmLmdldFRvQmVUaWxlZChlbGUucGFyZW50KClbMF0pKSkpXHJcbi8vICAgICAgICAgICAgcmV0dXJuIHRydWU7XHJcbi8vICAgICAgICBlbHNlXHJcbi8vICAgICAgICAgICAgcmV0dXJuIGZhbHNlO1xyXG4vLyAgICB9KTtcclxuLy9cclxuLy8gICAgLy8gQ3JlYXRlIGEgbWFwIG9mIHBhcmVudCBub2RlIGFuZCBpdHMgemVybyBkZWdyZWUgbWVtYmVyc1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IHplcm9EZWdyZWUubGVuZ3RoOyBpKyspXHJcbi8vICAgIHtcclxuLy8gICAgICAgIHZhciBub2RlID0gemVyb0RlZ3JlZVtpXTtcclxuLy8gICAgICAgIHZhciBwX2lkID0gbm9kZS5wYXJlbnQoKS5pZCgpO1xyXG4vL1xyXG4vLyAgICAgICAgaWYgKHBfaWQgIT0gdW5kZWZpbmVkICYmICFwYXJlbnRNYXBbcF9pZF0pIHtcclxuLy8gICAgICAgICAgICBwX2lkID0gdW5kZWZpbmVkO1xyXG4vLyAgICAgICAgfVxyXG4vL1xyXG4vLyAgICAgICAgaWYgKHR5cGVvZiB0ZW1wTWVtYmVyR3JvdXBzW3BfaWRdID09PSBcInVuZGVmaW5lZFwiKVxyXG4vLyAgICAgICAgICAgIHRlbXBNZW1iZXJHcm91cHNbcF9pZF0gPSBbXTtcclxuLy9cclxuLy8gICAgICAgIHRlbXBNZW1iZXJHcm91cHNbcF9pZF0gPSB0ZW1wTWVtYmVyR3JvdXBzW3BfaWRdLmNvbmNhdChub2RlKTtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICAvLyBJZiB0aGVyZSBhcmUgYXQgbGVhc3QgdHdvIG5vZGVzIGF0IGEgbGV2ZWwsIGNyZWF0ZSBhIGR1bW15IGNvbXBvdW5kIGZvciB0aGVtXHJcbi8vICAgIGZvciAodmFyIHBfaWQgaW4gdGVtcE1lbWJlckdyb3Vwcykge1xyXG4vLyAgICAgICAgaWYgKHRlbXBNZW1iZXJHcm91cHNbcF9pZF0ubGVuZ3RoID4gMSkge1xyXG4vLyAgICAgICAgICAgIHZhciBkdW1teUNvbXBvdW5kSWQgPSBcIkR1bW15Q29tcG91bmRfXCIgKyBwX2lkO1xyXG4vLyAgICAgICAgICAgIG1lbWJlckdyb3Vwc1tkdW1teUNvbXBvdW5kSWRdID0gdGVtcE1lbWJlckdyb3Vwc1twX2lkXTtcclxuLy9cclxuLy8gICAgICAgICAgICAvLyBDcmVhdGUgYSBkdW1teSBjb21wb3VuZFxyXG4vLyAgICAgICAgICAgIGlmICh0aGlzLm9wdGlvbnMuY3kuZ2V0RWxlbWVudEJ5SWQoZHVtbXlDb21wb3VuZElkKS5lbXB0eSgpKSB7XHJcbi8vICAgICAgICAgICAgICAgIHRoaXMub3B0aW9ucy5jeS5hZGQoe1xyXG4vLyAgICAgICAgICAgICAgICAgICAgZ3JvdXA6IFwibm9kZXNcIixcclxuLy8gICAgICAgICAgICAgICAgICAgIGRhdGE6IHtpZDogZHVtbXlDb21wb3VuZElkLCBwYXJlbnQ6IHBfaWRcclxuLy8gICAgICAgICAgICAgICAgICAgIH1cclxuLy8gICAgICAgICAgICAgICAgfSk7XHJcbi8vXHJcbi8vICAgICAgICAgICAgICAgIHZhciBkdW1teSA9IHRoaXMub3B0aW9ucy5jeS5ub2RlcygpW3RoaXMub3B0aW9ucy5jeS5ub2RlcygpLmxlbmd0aCAtIDFdO1xyXG4vLyAgICAgICAgICAgICAgICB0aGlzLm9wdGlvbnMuZWxlcyA9IHRoaXMub3B0aW9ucy5lbGVzLnVuaW9uKGR1bW15KTtcclxuLy8gICAgICAgICAgICAgICAgZHVtbXkuaGlkZSgpO1xyXG4vL1xyXG4vLyAgICAgICAgICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHRlbXBNZW1iZXJHcm91cHNbcF9pZF0ubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgICAgICAgICAgICAgIGlmIChpID09IDApIHtcclxuLy8gICAgICAgICAgICAgICAgICAgICAgICBkdW1teS5zY3JhdGNoKCdzYmduUGRMYXlvdXQnLCB7dGVtcGNoaWxkcmVuOiBbXX0pO1xyXG4vLyAgICAgICAgICAgICAgICAgICAgfVxyXG4vLyAgICAgICAgICAgICAgICAgICAgdmFyIG5vZGUgPSB0ZW1wTWVtYmVyR3JvdXBzW3BfaWRdW2ldO1xyXG4vLyAgICAgICAgICAgICAgICAgICAgdmFyIHNjcmF0Y2hPYmogPSBub2RlLnNjcmF0Y2goJ3NiZ25QZExheW91dCcpO1xyXG4vLyAgICAgICAgICAgICAgICAgICAgaWYgKCFzY3JhdGNoT2JqKSB7XHJcbi8vICAgICAgICAgICAgICAgICAgICAgICAgc2NyYXRjaE9iaiA9IHt9O1xyXG4vLyAgICAgICAgICAgICAgICAgICAgICAgIG5vZGUuc2NyYXRjaCgnc2JnblBkTGF5b3V0Jywgc2NyYXRjaE9iaik7XHJcbi8vICAgICAgICAgICAgICAgICAgICB9XHJcbi8vICAgICAgICAgICAgICAgICAgICBzY3JhdGNoT2JqWydkdW1teV9wYXJlbnRfaWQnXSA9IGR1bW15Q29tcG91bmRJZDtcclxuLy8gICAgICAgICAgICAgICAgICAgIHRoaXMub3B0aW9ucy5jeS5hZGQoe1xyXG4vLyAgICAgICAgICAgICAgICAgICAgICAgIGdyb3VwOiBcIm5vZGVzXCIsXHJcbi8vICAgICAgICAgICAgICAgICAgICAgICAgZGF0YToge3BhcmVudDogZHVtbXlDb21wb3VuZElkLCB3aWR0aDogbm9kZS53aWR0aCgpLCBoZWlnaHQ6IG5vZGUuaGVpZ2h0KClcclxuLy8gICAgICAgICAgICAgICAgICAgICAgICB9XHJcbi8vICAgICAgICAgICAgICAgICAgICB9KTtcclxuLy8gICAgICAgICAgICAgICAgICAgIHZhciB0ZW1wY2hpbGQgPSB0aGlzLm9wdGlvbnMuY3kubm9kZXMoKVt0aGlzLm9wdGlvbnMuY3kubm9kZXMoKS5sZW5ndGggLSAxXTtcclxuLy8gICAgICAgICAgICAgICAgICAgIHRlbXBjaGlsZC5oaWRlKCk7XHJcbi8vICAgICAgICAgICAgICAgICAgICB0ZW1wY2hpbGQuY3NzKCd3aWR0aCcsIHRlbXBjaGlsZC5kYXRhKCd3aWR0aCcpKTtcclxuLy8gICAgICAgICAgICAgICAgICAgIHRlbXBjaGlsZC5jc3MoJ2hlaWdodCcsIHRlbXBjaGlsZC5kYXRhKCdoZWlnaHQnKSk7XHJcbi8vICAgICAgICAgICAgICAgICAgICB0ZW1wY2hpbGQud2lkdGgoKTtcclxuLy8gICAgICAgICAgICAgICAgICAgIGR1bW15LnNjcmF0Y2goJ3NiZ25QZExheW91dCcpLnRlbXBjaGlsZHJlbi5wdXNoKHRlbXBjaGlsZCk7XHJcbi8vICAgICAgICAgICAgICAgIH1cclxuLy8gICAgICAgICAgICB9XHJcbi8vICAgICAgICB9XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgcmV0dXJuIG1lbWJlckdyb3VwcztcclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5wZXJmb3JtREZTT25Db21wb3VuZHMgPSBmdW5jdGlvbiAob3B0aW9ucykge1xyXG4vLyAgICB2YXIgY29tcG91bmRPcmRlciA9IFtdO1xyXG4vL1xyXG4vLyAgICB2YXIgcm9vdHMgPSBfU2JnblBETGF5b3V0LmdldFRvcE1vc3ROb2Rlcyh0aGlzLm9wdGlvbnMuZWxlcy5ub2RlcygpKTtcclxuLy8gICAgdGhpcy5maWxsQ29tcGV4T3JkZXJCeURGUyhjb21wb3VuZE9yZGVyLCByb290cyk7XHJcbi8vXHJcbi8vICAgIHJldHVybiBjb21wb3VuZE9yZGVyO1xyXG4vL307XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmZpbGxDb21wZXhPcmRlckJ5REZTID0gZnVuY3Rpb24gKGNvbXBvdW5kT3JkZXIsIGNoaWxkcmVuKSB7XHJcbi8vICAgIGZvciAodmFyIGkgPSAwOyBpIDwgY2hpbGRyZW4ubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgIHZhciBjaGlsZCA9IGNoaWxkcmVuW2ldO1xyXG4vLyAgICAgICAgdGhpcy5maWxsQ29tcGV4T3JkZXJCeURGUyhjb21wb3VuZE9yZGVyLCBjaGlsZC5jaGlsZHJlbigpKTtcclxuLy8gICAgICAgIGlmICh0aGlzLmdldFRvQmVUaWxlZChjaGlsZCkpIHtcclxuLy8gICAgICAgICAgICBjb21wb3VuZE9yZGVyLnB1c2goY2hpbGQpO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9XHJcbi8vfTtcclxuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuY2xlYXJDb21wb3VuZHMgPSBmdW5jdGlvbiAob3B0aW9ucykge1xyXG4vLyAgICB2YXIgY2hpbGRHcmFwaE1hcCA9IFtdO1xyXG4vL1xyXG4vLyAgICAvLyBHZXQgY29tcG91bmQgb3JkZXJpbmcgYnkgZmluZGluZyB0aGUgaW5uZXIgb25lIGZpcnN0XHJcbi8vICAgIHZhciBjb21wb3VuZE9yZGVyID0gdGhpcy5wZXJmb3JtREZTT25Db21wb3VuZHMob3B0aW9ucyk7XHJcbi8vICAgIF9TYmduUERMYXlvdXQuY29tcG91bmRPcmRlciA9IGNvbXBvdW5kT3JkZXI7XHJcbi8vICAgIHRoaXMucHJvY2Vzc0NoaWxkcmVuTGlzdCh0aGlzLnJvb3QsIF9TYmduUERMYXlvdXQuZ2V0VG9wTW9zdE5vZGVzKHRoaXMub3B0aW9ucy5lbGVzLm5vZGVzKCkpKTtcclxuLy9cclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBjb21wb3VuZE9yZGVyLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICAvLyBmaW5kIHRoZSBjb3JyZXNwb25kaW5nIGxheW91dCBub2RlXHJcbi8vICAgICAgICB2YXIgbENvbXBvdW5kTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2NvbXBvdW5kT3JkZXJbaV0uaWQoKV07XHJcbi8vXHJcbi8vICAgICAgICBjaGlsZEdyYXBoTWFwW2NvbXBvdW5kT3JkZXJbaV0uaWQoKV0gPSBjb21wb3VuZE9yZGVyW2ldLmNoaWxkcmVuKCk7XHJcbi8vXHJcbi8vICAgICAgICAvLyBSZW1vdmUgY2hpbGRyZW4gb2YgY29tcG91bmRzIFxyXG4vLyAgICAgICAgbENvbXBvdW5kTm9kZS5jaGlsZCA9IG51bGw7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy8gVGlsZSB0aGUgcmVtb3ZlZCBjaGlsZHJlblxyXG4vLyAgICB2YXIgdGlsZWRNZW1iZXJQYWNrID0gdGhpcy50aWxlQ29tcG91bmRNZW1iZXJzKGNoaWxkR3JhcGhNYXApO1xyXG4vL1xyXG4vLyAgICByZXR1cm4gdGlsZWRNZW1iZXJQYWNrO1xyXG4vL307XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmNsZWFyWmVyb0RlZ3JlZU1lbWJlcnMgPSBmdW5jdGlvbiAobWVtYmVyR3JvdXBzKSB7XHJcbi8vICAgIHZhciB0aWxlZFplcm9EZWdyZWVQYWNrID0gW107XHJcbi8vXHJcbi8vICAgIGZvciAodmFyIGlkIGluIG1lbWJlckdyb3Vwcykge1xyXG4vLyAgICAgICAgdmFyIGNvbXBvdW5kTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2lkXTtcclxuLy9cclxuLy8gICAgICAgIHRpbGVkWmVyb0RlZ3JlZVBhY2tbaWRdID0gdGhpcy50aWxlTm9kZXMobWVtYmVyR3JvdXBzW2lkXSk7XHJcbi8vXHJcbi8vICAgICAgICAvLyBTZXQgdGhlIHdpZHRoIGFuZCBoZWlnaHQgb2YgdGhlIGR1bW15IGNvbXBvdW5kIGFzIGNhbGN1bGF0ZWRcclxuLy8gICAgICAgIGNvbXBvdW5kTm9kZS5yZWN0LndpZHRoID0gdGlsZWRaZXJvRGVncmVlUGFja1tpZF0ud2lkdGg7XHJcbi8vICAgICAgICBjb21wb3VuZE5vZGUucmVjdC5oZWlnaHQgPSB0aWxlZFplcm9EZWdyZWVQYWNrW2lkXS5oZWlnaHQ7XHJcbi8vICAgIH1cclxuLy8gICAgcmV0dXJuIHRpbGVkWmVyb0RlZ3JlZVBhY2s7XHJcbi8vfTtcclxuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUucmVwb3B1bGF0ZUNvbXBvdW5kcyA9IGZ1bmN0aW9uICh0aWxlZE1lbWJlclBhY2spIHtcclxuLy8gICAgZm9yICh2YXIgaSA9IF9TYmduUERMYXlvdXQuY29tcG91bmRPcmRlci5sZW5ndGggLSAxOyBpID49IDA7IGktLSkge1xyXG4vLyAgICAgICAgdmFyIGlkID0gX1NiZ25QRExheW91dC5jb21wb3VuZE9yZGVyW2ldLmlkKCk7XHJcbi8vICAgICAgICB2YXIgbENvbXBvdW5kTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW2lkXTtcclxuLy8gICAgICAgIHZhciBob3Jpem9udGFsTWFyZ2luID0gcGFyc2VJbnQoX1NiZ25QRExheW91dC5jb21wb3VuZE9yZGVyW2ldLmNzcygncGFkZGluZy1sZWZ0JykpO1xyXG4vLyAgICAgICAgdmFyIHZlcnRpY2FsTWFyZ2luID0gcGFyc2VJbnQoX1NiZ25QRExheW91dC5jb21wb3VuZE9yZGVyW2ldLmNzcygncGFkZGluZy10b3AnKSk7XHJcbi8vXHJcbi8vICAgICAgICB0aGlzLmFkanVzdExvY2F0aW9ucyh0aWxlZE1lbWJlclBhY2tbaWRdLCBsQ29tcG91bmROb2RlLnJlY3QueCwgbENvbXBvdW5kTm9kZS5yZWN0LnksIGhvcml6b250YWxNYXJnaW4sIHZlcnRpY2FsTWFyZ2luKTtcclxuLy8gICAgfVxyXG4vL307XHJcbi8vXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUucmVwb3B1bGF0ZVplcm9EZWdyZWVNZW1iZXJzID0gZnVuY3Rpb24gKHRpbGVkUGFjaykge1xyXG4vLyAgICBmb3IgKHZhciBpIGluIHRpbGVkUGFjaykge1xyXG4vLyAgICAgICAgdmFyIGNvbXBvdW5kID0gdGhpcy5jeS5nZXRFbGVtZW50QnlJZChpKTtcclxuLy8gICAgICAgIHZhciBjb21wb3VuZE5vZGUgPSBfU2JnblBETGF5b3V0LmlkVG9MTm9kZVtpXTtcclxuLy8gICAgICAgIHZhciBob3Jpem9udGFsTWFyZ2luID0gcGFyc2VJbnQoY29tcG91bmQuY3NzKCdwYWRkaW5nLWxlZnQnKSk7XHJcbi8vICAgICAgICB2YXIgdmVydGljYWxNYXJnaW4gPSBwYXJzZUludChjb21wb3VuZC5jc3MoJ3BhZGRpbmctdG9wJykpO1xyXG4vL1xyXG4vLyAgICAgICAgLy8gQWRqdXN0IHRoZSBwb3NpdGlvbnMgb2Ygbm9kZXMgd3J0IGl0cyBjb21wb3VuZFxyXG4vLyAgICAgICAgdGhpcy5hZGp1c3RMb2NhdGlvbnModGlsZWRQYWNrW2ldLCBjb21wb3VuZE5vZGUucmVjdC54LCBjb21wb3VuZE5vZGUucmVjdC55LCBob3Jpem9udGFsTWFyZ2luLCB2ZXJ0aWNhbE1hcmdpbik7XHJcbi8vXHJcbi8vICAgICAgICB2YXIgdGVtcGNoaWxkcmVuID0gY29tcG91bmQuc2NyYXRjaCgnc2JnblBkTGF5b3V0JykudGVtcGNoaWxkcmVuO1xyXG4vLyAgICAgICAgZm9yICh2YXIgaSA9IDA7IGkgPCB0ZW1wY2hpbGRyZW4ubGVuZ3RoOyBpKyspIHtcclxuLy8gICAgICAgICAgICB0ZW1wY2hpbGRyZW5baV0ucmVtb3ZlKCk7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICAvLyBSZW1vdmUgdGhlIGR1bW15IGNvbXBvdW5kXHJcbi8vICAgICAgICBjb21wb3VuZC5yZW1vdmUoKTtcclxuLy8gICAgfVxyXG4vL307XHJcblxyXG4vKipcclxuICogVGhpcyBtZXRob2QgcGxhY2VzIGVhY2ggemVybyBkZWdyZWUgbWVtYmVyIHdydCBnaXZlbiAoeCx5KSBjb29yZGluYXRlcyAodG9wIGxlZnQpLiBcclxuICovXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuYWRqdXN0TG9jYXRpb25zID0gZnVuY3Rpb24gKG9yZ2FuaXphdGlvbiwgeCwgeSwgY29tcG91bmRIb3Jpem9udGFsTWFyZ2luLCBjb21wb3VuZFZlcnRpY2FsTWFyZ2luKSB7XHJcbi8vICAgIHggKz0gY29tcG91bmRIb3Jpem9udGFsTWFyZ2luO1xyXG4vLyAgICB5ICs9IGNvbXBvdW5kVmVydGljYWxNYXJnaW47XHJcbi8vXHJcbi8vICAgIHZhciBsZWZ0ID0geDtcclxuLy9cclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBvcmdhbml6YXRpb24ucm93cy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgdmFyIHJvdyA9IG9yZ2FuaXphdGlvbi5yb3dzW2ldO1xyXG4vLyAgICAgICAgeCA9IGxlZnQ7XHJcbi8vICAgICAgICB2YXIgbWF4SGVpZ2h0ID0gMDtcclxuLy9cclxuLy8gICAgICAgIGZvciAodmFyIGogPSAwOyBqIDwgcm93Lmxlbmd0aDsgaisrKSB7XHJcbi8vICAgICAgICAgICAgdmFyIGxub2RlID0gcm93W2pdO1xyXG4vLyAgICAgICAgICAgIHZhciBub2RlID0gdGhpcy5jeS5nZXRFbGVtZW50QnlJZChsbm9kZS5pZCk7XHJcbi8vXHJcbi8vICAgICAgICAgICAgbG5vZGUucmVjdC54ID0geDsvLyArIGxub2RlLnJlY3Qud2lkdGggLyAyO1xyXG4vLyAgICAgICAgICAgIGxub2RlLnJlY3QueSA9IHk7Ly8gKyBsbm9kZS5yZWN0LmhlaWdodCAvIDI7XHJcbi8vXHJcbi8vICAgICAgICAgICAgeCArPSBsbm9kZS5yZWN0LndpZHRoICsgb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nO1xyXG4vL1xyXG4vLyAgICAgICAgICAgIGlmIChsbm9kZS5yZWN0LmhlaWdodCA+IG1heEhlaWdodClcclxuLy8gICAgICAgICAgICAgICAgbWF4SGVpZ2h0ID0gbG5vZGUucmVjdC5oZWlnaHQ7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICB5ICs9IG1heEhlaWdodCArIG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vICAgIH1cclxuLy99O1xyXG5cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS50aWxlQ29tcG91bmRNZW1iZXJzID0gZnVuY3Rpb24gKGNoaWxkR3JhcGhNYXApIHtcclxuLy8gICAgdmFyIHRpbGVkTWVtYmVyUGFjayA9IFtdO1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpZCBpbiBjaGlsZEdyYXBoTWFwKSB7XHJcbi8vICAgICAgICAvLyBBY2Nlc3MgbGF5b3V0SW5mbyBub2RlcyB0byBzZXQgdGhlIHdpZHRoIGFuZCBoZWlnaHQgb2YgY29tcG91bmRzXHJcbi8vICAgICAgICB2YXIgY29tcG91bmROb2RlID0gX1NiZ25QRExheW91dC5pZFRvTE5vZGVbaWRdO1xyXG4vL1xyXG4vLyAgICAgICAgdGlsZWRNZW1iZXJQYWNrW2lkXSA9IHRoaXMudGlsZU5vZGVzKGNoaWxkR3JhcGhNYXBbaWRdKTtcclxuLy9cclxuLy8gICAgICAgIGNvbXBvdW5kTm9kZS5yZWN0LndpZHRoID0gdGlsZWRNZW1iZXJQYWNrW2lkXS53aWR0aCArIDIwO1xyXG4vLyAgICAgICAgY29tcG91bmROb2RlLnJlY3QuaGVpZ2h0ID0gdGlsZWRNZW1iZXJQYWNrW2lkXS5oZWlnaHQgKyAyMDtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICByZXR1cm4gdGlsZWRNZW1iZXJQYWNrO1xyXG4vL307XHJcblxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLnRpbGVOb2RlcyA9IGZ1bmN0aW9uIChub2Rlcykge1xyXG4vLyAgICB2YXIgc2VsZiA9IHRoaXM7XHJcbi8vICAgIHZhciB2ZXJ0aWNhbFBhZGRpbmcgPSB0eXBlb2Ygc2VsZi5vcHRpb25zLnRpbGluZ1BhZGRpbmdWZXJ0aWNhbCA9PT0gJ2Z1bmN0aW9uJyA/IHNlbGYub3B0aW9ucy50aWxpbmdQYWRkaW5nVmVydGljYWwuY2FsbCgpIDogc2VsZi5vcHRpb25zLnRpbGluZ1BhZGRpbmdWZXJ0aWNhbDtcclxuLy8gICAgdmFyIGhvcml6b250YWxQYWRkaW5nID0gdHlwZW9mIHNlbGYub3B0aW9ucy50aWxpbmdQYWRkaW5nSG9yaXpvbnRhbCA9PT0gJ2Z1bmN0aW9uJyA/IHNlbGYub3B0aW9ucy50aWxpbmdQYWRkaW5nSG9yaXpvbnRhbC5jYWxsKCkgOiBzZWxmLm9wdGlvbnMudGlsaW5nUGFkZGluZ0hvcml6b250YWw7XHJcbi8vICAgIHZhciBvcmdhbml6YXRpb24gPSB7XHJcbi8vICAgICAgICByb3dzOiBbXSxcclxuLy8gICAgICAgIHJvd1dpZHRoOiBbXSxcclxuLy8gICAgICAgIHJvd0hlaWdodDogW10sXHJcbi8vICAgICAgICB3aWR0aDogMjAsXHJcbi8vICAgICAgICBoZWlnaHQ6IDIwLFxyXG4vLyAgICAgICAgdmVydGljYWxQYWRkaW5nOiB2ZXJ0aWNhbFBhZGRpbmcsXHJcbi8vICAgICAgICBob3Jpem9udGFsUGFkZGluZzogaG9yaXpvbnRhbFBhZGRpbmdcclxuLy8gICAgfTtcclxuLy9cclxuLy8gICAgdmFyIGxheW91dE5vZGVzID0gW107XHJcbi8vXHJcbi8vICAgIC8vIEdldCBsYXlvdXQgbm9kZXNcclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBub2Rlcy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgdmFyIG5vZGUgPSBub2Rlc1tpXTtcclxuLy8gICAgICAgIHZhciBsTm9kZSA9IF9TYmduUERMYXlvdXQuaWRUb0xOb2RlW25vZGUuaWQoKV07XHJcbi8vXHJcbi8vICAgICAgICBpZiAoIW5vZGUuc2NyYXRjaCgnY29zZUJpbGtlbnQnKSB8fCAhbm9kZS5zY3JhdGNoKCdjb3NlQmlsa2VudCcpLmR1bW15X3BhcmVudF9pZCkge1xyXG4vLyAgICAgICAgICAgIHZhciBvd25lciA9IGxOb2RlLm93bmVyO1xyXG4vLyAgICAgICAgICAgIG93bmVyLnJlbW92ZShsTm9kZSk7XHJcbi8vXHJcbi8vICAgICAgICAgICAgdGhpcy5nbS5yZXNldEFsbE5vZGVzKCk7XHJcbi8vICAgICAgICAgICAgdGhpcy5nbS5nZXRBbGxOb2RlcygpO1xyXG4vLyAgICAgICAgfVxyXG4vL1xyXG4vLyAgICAgICAgbGF5b3V0Tm9kZXMucHVzaChsTm9kZSk7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy8gU29ydCB0aGUgbm9kZXMgaW4gYXNjZW5kaW5nIG9yZGVyIG9mIHRoZWlyIGFyZWFzXHJcbi8vICAgIGxheW91dE5vZGVzLnNvcnQoZnVuY3Rpb24gKG4xLCBuMikge1xyXG4vLyAgICAgICAgaWYgKG4xLnJlY3Qud2lkdGggKiBuMS5yZWN0LmhlaWdodCA+IG4yLnJlY3Qud2lkdGggKiBuMi5yZWN0LmhlaWdodClcclxuLy8gICAgICAgICAgICByZXR1cm4gLTE7XHJcbi8vICAgICAgICBpZiAobjEucmVjdC53aWR0aCAqIG4xLnJlY3QuaGVpZ2h0IDwgbjIucmVjdC53aWR0aCAqIG4yLnJlY3QuaGVpZ2h0KVxyXG4vLyAgICAgICAgICAgIHJldHVybiAxO1xyXG4vLyAgICAgICAgcmV0dXJuIDA7XHJcbi8vICAgIH0pO1xyXG4vL1xyXG4vLyAgICAvLyBDcmVhdGUgdGhlIG9yZ2FuaXphdGlvbiAtPiB0aWxlIG1lbWJlcnNcclxuLy8gICAgZm9yICh2YXIgaSA9IDA7IGkgPCBsYXlvdXROb2Rlcy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgdmFyIGxOb2RlID0gbGF5b3V0Tm9kZXNbaV07XHJcbi8vXHJcbi8vICAgICAgICB2YXIgY3lOb2RlID0gdGhpcy5jeS5nZXRFbGVtZW50QnlJZChsTm9kZS5pZCkucGFyZW50KClbMF07XHJcbi8vICAgICAgICB2YXIgbWluV2lkdGggPSAwO1xyXG4vLyAgICAgICAgaWYgKGN5Tm9kZSkge1xyXG4vLyAgICAgICAgICAgIG1pbldpZHRoID0gcGFyc2VJbnQoY3lOb2RlLmNzcygncGFkZGluZy1sZWZ0JykpICsgcGFyc2VJbnQoY3lOb2RlLmNzcygncGFkZGluZy1yaWdodCcpKTtcclxuLy8gICAgICAgIH1cclxuLy9cclxuLy8gICAgICAgIGlmIChvcmdhbml6YXRpb24ucm93cy5sZW5ndGggPT0gMCkge1xyXG4vLyAgICAgICAgICAgIHRoaXMuaW5zZXJ0Tm9kZVRvUm93KG9yZ2FuaXphdGlvbiwgbE5vZGUsIDAsIG1pbldpZHRoKTtcclxuLy8gICAgICAgIH0gZWxzZSBpZiAodGhpcy5jYW5BZGRIb3Jpem9udGFsKG9yZ2FuaXphdGlvbiwgbE5vZGUucmVjdC53aWR0aCwgbE5vZGUucmVjdC5oZWlnaHQpKSB7XHJcbi8vICAgICAgICAgICAgdGhpcy5pbnNlcnROb2RlVG9Sb3cob3JnYW5pemF0aW9uLCBsTm9kZSwgdGhpcy5nZXRTaG9ydGVzdFJvd0luZGV4KG9yZ2FuaXphdGlvbiksIG1pbldpZHRoKTtcclxuLy8gICAgICAgIH0gZWxzZSB7XHJcbi8vICAgICAgICAgICAgdGhpcy5pbnNlcnROb2RlVG9Sb3cob3JnYW5pemF0aW9uLCBsTm9kZSwgb3JnYW5pemF0aW9uLnJvd3MubGVuZ3RoLCBtaW5XaWR0aCk7XHJcbi8vICAgICAgICB9XHJcbi8vXHJcbi8vICAgICAgICB0aGlzLnNoaWZ0VG9MYXN0Um93KG9yZ2FuaXphdGlvbik7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgcmV0dXJuIG9yZ2FuaXphdGlvbjtcclxuLy99O1xyXG4vL1xyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmluc2VydE5vZGVUb1JvdyA9IGZ1bmN0aW9uIChvcmdhbml6YXRpb24sIG5vZGUsIHJvd0luZGV4LCBtaW5XaWR0aCkge1xyXG4vLyAgICB2YXIgbWluQ29tcG91bmRTaXplID0gbWluV2lkdGg7XHJcbi8vXHJcbi8vICAgIC8vIEFkZCBuZXcgcm93IGlmIG5lZWRlZFxyXG4vLyAgICBpZiAocm93SW5kZXggPT0gb3JnYW5pemF0aW9uLnJvd3MubGVuZ3RoKSB7XHJcbi8vICAgICAgICB2YXIgc2Vjb25kRGltZW5zaW9uID0gW107XHJcbi8vXHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93cy5wdXNoKHNlY29uZERpbWVuc2lvbik7XHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93V2lkdGgucHVzaChtaW5Db21wb3VuZFNpemUpO1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLnJvd0hlaWdodC5wdXNoKDApO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIC8vIFVwZGF0ZSByb3cgd2lkdGhcclxuLy8gICAgdmFyIHcgPSBvcmdhbml6YXRpb24ucm93V2lkdGhbcm93SW5kZXhdICsgbm9kZS5yZWN0LndpZHRoO1xyXG4vL1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLnJvd3Nbcm93SW5kZXhdLmxlbmd0aCA+IDApIHtcclxuLy8gICAgICAgIHcgKz0gb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtyb3dJbmRleF0gPSB3O1xyXG4vLyAgICAvLyBVcGRhdGUgY29tcG91bmQgd2lkdGhcclxuLy8gICAgaWYgKG9yZ2FuaXphdGlvbi53aWR0aCA8IHcpIHtcclxuLy8gICAgICAgIG9yZ2FuaXphdGlvbi53aWR0aCA9IHc7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgLy8gVXBkYXRlIGhlaWdodFxyXG4vLyAgICB2YXIgaCA9IG5vZGUucmVjdC5oZWlnaHQ7XHJcbi8vICAgIGlmIChyb3dJbmRleCA+IDApXHJcbi8vICAgICAgICBoICs9IG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vXHJcbi8vICAgIHZhciBleHRyYUhlaWdodCA9IDA7XHJcbi8vICAgIGlmIChoID4gb3JnYW5pemF0aW9uLnJvd0hlaWdodFtyb3dJbmRleF0pIHtcclxuLy8gICAgICAgIGV4dHJhSGVpZ2h0ID0gb3JnYW5pemF0aW9uLnJvd0hlaWdodFtyb3dJbmRleF07XHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93SGVpZ2h0W3Jvd0luZGV4XSA9IGg7XHJcbi8vICAgICAgICBleHRyYUhlaWdodCA9IG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbcm93SW5kZXhdIC0gZXh0cmFIZWlnaHQ7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgb3JnYW5pemF0aW9uLmhlaWdodCArPSBleHRyYUhlaWdodDtcclxuLy9cclxuLy8gICAgLy8gSW5zZXJ0IG5vZGVcclxuLy8gICAgb3JnYW5pemF0aW9uLnJvd3Nbcm93SW5kZXhdLnB1c2gobm9kZSk7XHJcbi8vfTtcclxuLy9cclxuLy8vL1NjYW5zIHRoZSByb3dzIG9mIGFuIG9yZ2FuaXphdGlvbiBhbmQgcmV0dXJucyB0aGUgb25lIHdpdGggdGhlIG1pbiB3aWR0aFxyXG4vL19TYmduUERMYXlvdXQucHJvdG90eXBlLmdldFNob3J0ZXN0Um93SW5kZXggPSBmdW5jdGlvbiAob3JnYW5pemF0aW9uKSB7XHJcbi8vICAgIHZhciByID0gLTE7XHJcbi8vICAgIHZhciBtaW4gPSBOdW1iZXIuTUFYX1ZBTFVFO1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IG9yZ2FuaXphdGlvbi5yb3dzLmxlbmd0aDsgaSsrKSB7XHJcbi8vICAgICAgICBpZiAob3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldIDwgbWluKSB7XHJcbi8vICAgICAgICAgICAgciA9IGk7XHJcbi8vICAgICAgICAgICAgbWluID0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9XHJcbi8vICAgIHJldHVybiByO1xyXG4vL307XHJcbi8vXHJcbi8vLy9TY2FucyB0aGUgcm93cyBvZiBhbiBvcmdhbml6YXRpb24gYW5kIHJldHVybnMgdGhlIG9uZSB3aXRoIHRoZSBtYXggd2lkdGhcclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5nZXRMb25nZXN0Um93SW5kZXggPSBmdW5jdGlvbiAob3JnYW5pemF0aW9uKSB7XHJcbi8vICAgIHZhciByID0gLTE7XHJcbi8vICAgIHZhciBtYXggPSBOdW1iZXIuTUlOX1ZBTFVFO1xyXG4vL1xyXG4vLyAgICBmb3IgKHZhciBpID0gMDsgaSA8IG9yZ2FuaXphdGlvbi5yb3dzLmxlbmd0aDsgaSsrKSB7XHJcbi8vXHJcbi8vICAgICAgICBpZiAob3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldID4gbWF4KSB7XHJcbi8vICAgICAgICAgICAgciA9IGk7XHJcbi8vICAgICAgICAgICAgbWF4ID0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW2ldO1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIHJldHVybiByO1xyXG4vL307XHJcbi8vXHJcbi8vLyoqXHJcbi8vICogVGhpcyBtZXRob2QgY2hlY2tzIHdoZXRoZXIgYWRkaW5nIGV4dHJhIHdpZHRoIHRvIHRoZSBvcmdhbml6YXRpb24gdmlvbGF0ZXNcclxuLy8gKiB0aGUgYXNwZWN0IHJhdGlvKDEpIG9yIG5vdC5cclxuLy8gKi9cclxuLy9fU2JnblBETGF5b3V0LnByb3RvdHlwZS5jYW5BZGRIb3Jpem9udGFsID0gZnVuY3Rpb24gKG9yZ2FuaXphdGlvbiwgZXh0cmFXaWR0aCwgZXh0cmFIZWlnaHQpIHtcclxuLy9cclxuLy8gICAgdmFyIHNyaSA9IHRoaXMuZ2V0U2hvcnRlc3RSb3dJbmRleChvcmdhbml6YXRpb24pO1xyXG4vL1xyXG4vLyAgICBpZiAoc3JpIDwgMCkge1xyXG4vLyAgICAgICAgcmV0dXJuIHRydWU7XHJcbi8vICAgIH1cclxuLy9cclxuLy8gICAgdmFyIG1pbiA9IG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtzcmldO1xyXG4vL1xyXG4vLyAgICBpZiAobWluICsgb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nICsgZXh0cmFXaWR0aCA8PSBvcmdhbml6YXRpb24ud2lkdGgpXHJcbi8vICAgICAgICByZXR1cm4gdHJ1ZTtcclxuLy9cclxuLy8gICAgdmFyIGhEaWZmID0gMDtcclxuLy9cclxuLy8gICAgLy8gQWRkaW5nIHRvIGFuIGV4aXN0aW5nIHJvd1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLnJvd0hlaWdodFtzcmldIDwgZXh0cmFIZWlnaHQpIHtcclxuLy8gICAgICAgIGlmIChzcmkgPiAwKVxyXG4vLyAgICAgICAgICAgIGhEaWZmID0gZXh0cmFIZWlnaHQgKyBvcmdhbml6YXRpb24udmVydGljYWxQYWRkaW5nIC0gb3JnYW5pemF0aW9uLnJvd0hlaWdodFtzcmldO1xyXG4vLyAgICB9XHJcbi8vXHJcbi8vICAgIHZhciBhZGRfdG9fcm93X3JhdGlvO1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLndpZHRoIC0gbWluID49IGV4dHJhV2lkdGggKyBvcmdhbml6YXRpb24uaG9yaXpvbnRhbFBhZGRpbmcpIHtcclxuLy8gICAgICAgIGFkZF90b19yb3dfcmF0aW8gPSAob3JnYW5pemF0aW9uLmhlaWdodCArIGhEaWZmKSAvIChtaW4gKyBleHRyYVdpZHRoICsgb3JnYW5pemF0aW9uLmhvcml6b250YWxQYWRkaW5nKTtcclxuLy8gICAgfSBlbHNlIHtcclxuLy8gICAgICAgIGFkZF90b19yb3dfcmF0aW8gPSAob3JnYW5pemF0aW9uLmhlaWdodCArIGhEaWZmKSAvIG9yZ2FuaXphdGlvbi53aWR0aDtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICAvLyBBZGRpbmcgYSBuZXcgcm93IGZvciB0aGlzIG5vZGVcclxuLy8gICAgaERpZmYgPSBleHRyYUhlaWdodCArIG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vICAgIHZhciBhZGRfbmV3X3Jvd19yYXRpbztcclxuLy8gICAgaWYgKG9yZ2FuaXphdGlvbi53aWR0aCA8IGV4dHJhV2lkdGgpIHtcclxuLy8gICAgICAgIGFkZF9uZXdfcm93X3JhdGlvID0gKG9yZ2FuaXphdGlvbi5oZWlnaHQgKyBoRGlmZikgLyBleHRyYVdpZHRoO1xyXG4vLyAgICB9IGVsc2Uge1xyXG4vLyAgICAgICAgYWRkX25ld19yb3dfcmF0aW8gPSAob3JnYW5pemF0aW9uLmhlaWdodCArIGhEaWZmKSAvIG9yZ2FuaXphdGlvbi53aWR0aDtcclxuLy8gICAgfVxyXG4vL1xyXG4vLyAgICBpZiAoYWRkX25ld19yb3dfcmF0aW8gPCAxKVxyXG4vLyAgICAgICAgYWRkX25ld19yb3dfcmF0aW8gPSAxIC8gYWRkX25ld19yb3dfcmF0aW87XHJcbi8vXHJcbi8vICAgIGlmIChhZGRfdG9fcm93X3JhdGlvIDwgMSlcclxuLy8gICAgICAgIGFkZF90b19yb3dfcmF0aW8gPSAxIC8gYWRkX3RvX3Jvd19yYXRpbztcclxuLy9cclxuLy8gICAgcmV0dXJuIGFkZF90b19yb3dfcmF0aW8gPCBhZGRfbmV3X3Jvd19yYXRpbztcclxuLy99O1xyXG4vL1xyXG4vL1xyXG4vLy8vSWYgbW92aW5nIHRoZSBsYXN0IG5vZGUgZnJvbSB0aGUgbG9uZ2VzdCByb3cgYW5kIGFkZGluZyBpdCB0byB0aGUgbGFzdFxyXG4vLy8vcm93IG1ha2VzIHRoZSBib3VuZGluZyBib3ggc21hbGxlciwgZG8gaXQuXHJcbi8vX1NiZ25QRExheW91dC5wcm90b3R5cGUuc2hpZnRUb0xhc3RSb3cgPSBmdW5jdGlvbiAob3JnYW5pemF0aW9uKSB7XHJcbi8vICAgIHZhciBsb25nZXN0ID0gdGhpcy5nZXRMb25nZXN0Um93SW5kZXgob3JnYW5pemF0aW9uKTtcclxuLy8gICAgdmFyIGxhc3QgPSBvcmdhbml6YXRpb24ucm93V2lkdGgubGVuZ3RoIC0gMTtcclxuLy8gICAgdmFyIHJvdyA9IG9yZ2FuaXphdGlvbi5yb3dzW2xvbmdlc3RdO1xyXG4vLyAgICB2YXIgbm9kZSA9IHJvd1tyb3cubGVuZ3RoIC0gMV07XHJcbi8vXHJcbi8vICAgIHZhciBkaWZmID0gbm9kZS53aWR0aCArIG9yZ2FuaXphdGlvbi5ob3Jpem9udGFsUGFkZGluZztcclxuLy9cclxuLy8gICAgLy8gQ2hlY2sgaWYgdGhlcmUgaXMgZW5vdWdoIHNwYWNlIG9uIHRoZSBsYXN0IHJvd1xyXG4vLyAgICBpZiAob3JnYW5pemF0aW9uLndpZHRoIC0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW2xhc3RdID4gZGlmZiAmJiBsb25nZXN0ICE9IGxhc3QpIHtcclxuLy8gICAgICAgIC8vIFJlbW92ZSB0aGUgbGFzdCBlbGVtZW50IG9mIHRoZSBsb25nZXN0IHJvd1xyXG4vLyAgICAgICAgcm93LnNwbGljZSgtMSwgMSk7XHJcbi8vXHJcbi8vICAgICAgICAvLyBQdXNoIGl0IHRvIHRoZSBsYXN0IHJvd1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLnJvd3NbbGFzdF0ucHVzaChub2RlKTtcclxuLy9cclxuLy8gICAgICAgIG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtsb25nZXN0XSA9IG9yZ2FuaXphdGlvbi5yb3dXaWR0aFtsb25nZXN0XSAtIGRpZmY7XHJcbi8vICAgICAgICBvcmdhbml6YXRpb24ucm93V2lkdGhbbGFzdF0gPSBvcmdhbml6YXRpb24ucm93V2lkdGhbbGFzdF0gKyBkaWZmO1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLndpZHRoID0gb3JnYW5pemF0aW9uLnJvd1dpZHRoW3RoaXMuZ2V0TG9uZ2VzdFJvd0luZGV4KG9yZ2FuaXphdGlvbildO1xyXG4vL1xyXG4vLyAgICAgICAgLy8gVXBkYXRlIGhlaWdodHMgb2YgdGhlIG9yZ2FuaXphdGlvblxyXG4vLyAgICAgICAgdmFyIG1heEhlaWdodCA9IE51bWJlci5NSU5fVkFMVUU7XHJcbi8vICAgICAgICBmb3IgKHZhciBpID0gMDsgaSA8IHJvdy5sZW5ndGg7IGkrKykge1xyXG4vLyAgICAgICAgICAgIGlmIChyb3dbaV0uaGVpZ2h0ID4gbWF4SGVpZ2h0KVxyXG4vLyAgICAgICAgICAgICAgICBtYXhIZWlnaHQgPSByb3dbaV0uaGVpZ2h0O1xyXG4vLyAgICAgICAgfVxyXG4vLyAgICAgICAgaWYgKGxvbmdlc3QgPiAwKVxyXG4vLyAgICAgICAgICAgIG1heEhlaWdodCArPSBvcmdhbml6YXRpb24udmVydGljYWxQYWRkaW5nO1xyXG4vL1xyXG4vLyAgICAgICAgdmFyIHByZXZUb3RhbCA9IG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbbG9uZ2VzdF0gKyBvcmdhbml6YXRpb24ucm93SGVpZ2h0W2xhc3RdO1xyXG4vL1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLnJvd0hlaWdodFtsb25nZXN0XSA9IG1heEhlaWdodDtcclxuLy8gICAgICAgIGlmIChvcmdhbml6YXRpb24ucm93SGVpZ2h0W2xhc3RdIDwgbm9kZS5oZWlnaHQgKyBvcmdhbml6YXRpb24udmVydGljYWxQYWRkaW5nKVxyXG4vLyAgICAgICAgICAgIG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbbGFzdF0gPSBub2RlLmhlaWdodCArIG9yZ2FuaXphdGlvbi52ZXJ0aWNhbFBhZGRpbmc7XHJcbi8vXHJcbi8vICAgICAgICB2YXIgZmluYWxUb3RhbCA9IG9yZ2FuaXphdGlvbi5yb3dIZWlnaHRbbG9uZ2VzdF0gKyBvcmdhbml6YXRpb24ucm93SGVpZ2h0W2xhc3RdO1xyXG4vLyAgICAgICAgb3JnYW5pemF0aW9uLmhlaWdodCArPSAoZmluYWxUb3RhbCAtIHByZXZUb3RhbCk7XHJcbi8vXHJcbi8vICAgICAgICB0aGlzLnNoaWZ0VG9MYXN0Um93KG9yZ2FuaXphdGlvbik7XHJcbi8vICAgIH1cclxuLy99O1xyXG5cclxuLyoqXHJcbiAqIEBicmllZiA6IGNhbGxlZCBvbiBjb250aW51b3VzIGxheW91dHMgdG8gc3RvcCB0aGVtIGJlZm9yZSB0aGV5IGZpbmlzaFxyXG4gKi9cclxuX1NiZ25QRExheW91dC5wcm90b3R5cGUuc3RvcCA9IGZ1bmN0aW9uICgpIHtcclxuICAgIHRoaXMuc3RvcHBlZCA9IHRydWU7XHJcblxyXG4gICAgaWYgKHRoaXMudGhyZWFkKSB7XHJcbiAgICAgICAgdGhpcy50aHJlYWQuc3RvcCgpO1xyXG4gICAgfVxyXG5cclxuICAgIHRoaXMudHJpZ2dlcignbGF5b3V0c3RvcCcpO1xyXG5cclxuICAgIHJldHVybiB0aGlzOyAvLyBjaGFpbmluZ1xyXG59O1xyXG5cclxuX1NiZ25QRExheW91dC5wcm90b3R5cGUucHJvY2Vzc0NoaWxkcmVuTGlzdCA9IGZ1bmN0aW9uIChwYXJlbnQsIGNoaWxkcmVuKSB7XHJcbiAgICB2YXIgc2l6ZSA9IGNoaWxkcmVuLmxlbmd0aDtcclxuICAgIGZvciAodmFyIGkgPSAwOyBpIDwgc2l6ZTsgaSsrKSB7XHJcbiAgICAgICAgdmFyIHRoZUNoaWxkID0gY2hpbGRyZW5baV07XHJcbiAgICAgICAgdGhpcy5vcHRpb25zLmVsZXMubm9kZXMoKS5sZW5ndGg7XHJcbiAgICAgICAgdmFyIGNoaWxkcmVuX29mX2NoaWxkcmVuID0gdGhlQ2hpbGQuY2hpbGRyZW4oKTtcclxuICAgICAgICB2YXIgdGhlTm9kZTtcclxuXHJcbiAgICAgICAgaWYgKHRoZUNoaWxkLndpZHRoKCkgIT0gbnVsbFxyXG4gICAgICAgICAgICAgICAgJiYgdGhlQ2hpbGQuaGVpZ2h0KCkgIT0gbnVsbCkge1xyXG4gICAgICAgICAgICB0aGVOb2RlID0gcGFyZW50LmFkZChuZXcgU2JnblBETm9kZShfU2JnblBETGF5b3V0LmxheW91dC5ncmFwaE1hbmFnZXIsXHJcbiAgICAgICAgICAgICAgICAgICAgbmV3IFBvaW50RCh0aGVDaGlsZC5wb3NpdGlvbigneCcpLCB0aGVDaGlsZC5wb3NpdGlvbigneScpKSxcclxuICAgICAgICAgICAgICAgICAgICBuZXcgRGltZW5zaW9uRChwYXJzZUZsb2F0KHRoZUNoaWxkLndpZHRoKCkpLFxyXG4gICAgICAgICAgICAgICAgICAgICAgICAgICAgcGFyc2VGbG9hdCh0aGVDaGlsZC5oZWlnaHQoKSkpKSk7XHJcbiAgICAgICAgfSBlbHNlIHtcclxuICAgICAgICAgICAgdGhlTm9kZSA9IHBhcmVudC5hZGQobmV3IFNiZ25QRE5vZGUodGhpcy5ncmFwaE1hbmFnZXIpKTtcclxuICAgICAgICB9XHJcbiAgICAgICAgdGhlTm9kZS5pZCA9IHRoZUNoaWxkLmRhdGEoXCJpZFwiKTtcclxuICAgICAgICBfU2JnblBETGF5b3V0LmlkVG9MTm9kZVt0aGVDaGlsZC5kYXRhKFwiaWRcIildID0gdGhlTm9kZTtcclxuXHJcbiAgICAgICAgaWYgKGlzTmFOKHRoZU5vZGUucmVjdC54KSkge1xyXG4gICAgICAgICAgICB0aGVOb2RlLnJlY3QueCA9IDA7XHJcbiAgICAgICAgfVxyXG5cclxuICAgICAgICBpZiAoaXNOYU4odGhlTm9kZS5yZWN0LnkpKSB7XHJcbiAgICAgICAgICAgIHRoZU5vZGUucmVjdC55ID0gMDtcclxuICAgICAgICB9XHJcblxyXG4gICAgICAgIGlmIChjaGlsZHJlbl9vZl9jaGlsZHJlbiAhPSBudWxsICYmIGNoaWxkcmVuX29mX2NoaWxkcmVuLmxlbmd0aCA+IDApIHtcclxuICAgICAgICAgICAgdmFyIHRoZU5ld0dyYXBoO1xyXG4gICAgICAgICAgICB0aGVOZXdHcmFwaCA9IF9TYmduUERMYXlvdXQubGF5b3V0LmdldEdyYXBoTWFuYWdlcigpLmFkZChfU2JnblBETGF5b3V0LmxheW91dC5uZXdHcmFwaCgpLCB0aGVOb2RlKTtcclxuICAgICAgICAgICAgdGhpcy5wcm9jZXNzQ2hpbGRyZW5MaXN0KHRoZU5ld0dyYXBoLCBjaGlsZHJlbl9vZl9jaGlsZHJlbik7XHJcbiAgICAgICAgfVxyXG4gICAgfVxyXG59O1xyXG5cclxubW9kdWxlLmV4cG9ydHMgPSBmdW5jdGlvbiBnZXQoY3l0b3NjYXBlKSB7XHJcbiAgICBUaHJlYWQgPSBjeXRvc2NhcGUuVGhyZWFkO1xyXG5cclxuICAgIHJldHVybiBfU2JnblBETGF5b3V0O1xyXG59O1xyXG4iLCIndXNlIHN0cmljdCc7XHJcblxyXG4vLyByZWdpc3RlcnMgdGhlIGV4dGVuc2lvbiBvbiBhIGN5dG9zY2FwZSBsaWIgcmVmXHJcbnZhciBnZXRMYXlvdXQgPSByZXF1aXJlKCcuL0xheW91dCcpO1xyXG4vL3ZhciBnZXRVdGlsaXRpZXMgPSByZXF1aXJlKCcuL1V0aWxpdGllcycpO1xyXG5cclxudmFyIHJlZ2lzdGVyID0gZnVuY3Rpb24oIGN5dG9zY2FwZSApe1xyXG4gIHZhciBMYXlvdXQgPSBnZXRMYXlvdXQoIGN5dG9zY2FwZSApO1xyXG4gIC8vdmFyIFV0aWxpdGllcyA9IGdldFV0aWxpdGllcyAoIGN5dG9zY2FwZSApO1xyXG4gIFxyXG4gIGN5dG9zY2FwZSgnbGF5b3V0JywgJ3NiZ25QZExheW91dCcsIExheW91dCk7XHJcbiAgLy9jeXRvc2NhcGUoJ2NvcmUnLCAndXRpbGl0aWVzJywgVXRpbGl0aWVzKTtcclxufTtcclxuXHJcbmlmKCB0eXBlb2YgY3l0b3NjYXBlICE9PSAndW5kZWZpbmVkJyApeyAvLyBleHBvc2UgdG8gZ2xvYmFsIGN5dG9zY2FwZSAoaS5lLiB3aW5kb3cuY3l0b3NjYXBlKVxyXG4gIHJlZ2lzdGVyKCBjeXRvc2NhcGUgKTtcclxufVxyXG5cclxubW9kdWxlLmV4cG9ydHMgPSByZWdpc3RlcjtcclxuIl19
