var PolyominoQuickSort = require('./PolyominoQuickSort');
var Integer = require('./Integer');
var RectangleD = require('./RectangleD');

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