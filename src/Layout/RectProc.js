var RandomSeed = require('./RandomSeed');

RectProc.AspectRatio = (1.0 / 1.0);// ysize/xsize

function RectProc() {
}

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
    Polyomino[] minos = new Polyomino[rN];

    for (int i = 0; i < rN; i++)
    {
            // size of the rectangle in grid units
            int W = (int) Math.ceil((rW[i] + buffer) / stepX);
            int H = (int) Math.ceil((rH[i] + buffer) / stepY);

            minos[i] = new Polyomino();
            minos[i].coord = new Point[W * H];

            // create the polyomino cells
            int cnt = 0;
            for (int y = 0; y < H; y++)
                    for (int x = 0; x < W; x++)
                    {
                            minos[i].coord[cnt] = new Point();
                            minos[i].coord[cnt].x = x;
                            minos[i].coord[cnt++].y = y;

                    }
            minos[i].l = cnt;
            minos[i].label = rectangles[i].label;
    }

    // do the packing
    PolyominoPacking packer = new PolyominoPacking();
    packer.pack(minos, rN);

    // get the results
    for (int i = 0; i < rN; i++)
    {
            rX[i] = minos[i].x * stepX;
            rY[i] = minos[i].y * stepY;
    }
};

module.exports = RectProc;