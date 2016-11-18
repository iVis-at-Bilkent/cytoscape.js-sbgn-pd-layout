var CoSELayout = require('./CoSELayout');
var HashMap = require('./HashMap');
var SbgnPDConstants = require('./SbgnPDConstants');
var IGeometry = require('./IGeometry');
var PointD = require('./PointD');
var SbgnPDConstants = require('./SbgnPDConstants');

SbgnPDLayout.prototype.DefaultCompactionAlgorithmEnum = 
{
    TILING : 0, 
    POLYOMINO_PACKING : 1
};

SbgnPDLayout.prototype.OrientationEnum = 
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
    this.compactionMethod = this.DefaultCompactionAlgorithmEnum.TILING;

    this.childGraphMap = new HashMap();          /*Map<SbgnPDNode, LGraph>*/
    this.complexOrder = [];                      /*List<SbgnPDNode>*/
    this.dummyComplexList = [];                  /*List<SbgnPDNode>*/
    this.emptiedDummyComplexMap = new HashMap(); /*Map<SbgnPDNode, LGraph>*/
    this.processNodeList = [];                   /*List<SbgnProcessNode>*/
    
    if (this.compactionMethod === this.DefaultCompactionAlgorithmEnum.TILING)
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

    orientationList.push(this.Orientation.LEFT_TO_RIGHT);
    orientationList.push(this.Orientation.RIGHT_TO_LEFT);
    orientationList.push(this.Orientation.TOP_TO_BOTTOM);
    orientationList.push(this.Orientation.BOTTOM_TO_TOP);
    
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
    if (orient === Orientation.LEFT_TO_RIGHT || 
        orient === Orientation.RIGHT_TO_LEFT)
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
    else if (orient === Orientation.BOTTOM_TO_TOP || 
             orient === Orientation.TOP_TO_BOTTOM)
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
        degree = parentNode.getEdges().size();
        return degree;
    }

    var numOfChildren = parentNode.getChild().getNodes().length;
    for (var i=0; i<numOfChildren; i++)
    {
        degree = degree + parentNode.getEdges().length
                        + calcGraphDegree(parentNode.getChild().getNodes()[i]);
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
        if ((ownerGraph.getParent().type != null) && 
            (ownerGraph.getParent().isComplex()))
        {
            continue;
        }
        
        var numOfNodes = ownerGraph.getNodes().length;
        for (var j=0; j<numOfNodes; j++)
        {
            var node = ownerGraph.getNodes()[j];

            if (calcGraphDegree(node) == 0)
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

module.exports = SbgnPDLayout;
