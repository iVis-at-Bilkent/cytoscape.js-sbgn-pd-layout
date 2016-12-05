var Integer = require('./Integer');
var IGeometry = require('./IGeometry');
var PointD = require('./PointD');
var RectangleD = require('./RectangleD');

var HashMap = require('./HashMap');
var HashSet = require('./HashSet');

var CoSELayout = require('./CoSELayout');
var SbgnPDNode = require('./SbgnPDNode');
var SbgnPDEdge = require('./SbgnPDEdge');
var SbgnProcessNode = require('./SbgnProcessNode');
var SbgnPDConstants = require('./SbgnPDConstants');

var MemberPack = require('./MemberPack');
var RectProc = require('./RectProc');
var Compaction = require('./Compaction');

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
