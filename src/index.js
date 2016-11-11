'use strict';

// registers the extension on a cytoscape lib ref
var getLayout = require('./Layout');
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
