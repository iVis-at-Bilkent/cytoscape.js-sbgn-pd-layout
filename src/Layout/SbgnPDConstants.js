var CoSEConstants = require('./CoSEConstants');

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