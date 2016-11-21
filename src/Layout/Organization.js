var SbgnPDConstants = require('./SbgnPDConstants');

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
