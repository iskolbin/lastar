local astar = require('astar')
local floor = math.floor

local width, height = 10, 10
local function makelevel( floorcost, wallcost ) 
	local _, W = floorcost, wallcost
	return {
		_, _, _, _, _, _, _, _, _, _,
		_, W, W, W, W, W, W, W, W, _,
		_, _, _, _, _, _, _, _, W, _,
		_, _, _, _, _, _, _, _, W, _,
		_, _, _, _, _, _, _, _, W, _,
		_, _, _, _, _, _, _, _, W, _,
		_, _, _, _, _, _, _, _, W, _,
		_, _, _, _, _, _, _, _, W, _,
		_, _, _, _, _, _, _, _, W, _,
		_, _, _, _, _, _, _, _, _, _
	}
end

local function id2xy( id )
	return ( (id-1) % width ) + 1, floor( (id-1) / width ) + 1
end

local function xy2id( x, y )
	return x + (y-1)*width
end

local function xy2cost( level, x, y )
	return level[xy2id( x, y )]
end

local function addnodeif( predicate, neighbors, level, x, y, n, mul )
	if predicate then
		local cost = xy2cost( level, x, y ) * (mul or 1)
		if cost < math.huge then
			neighbors[xy2id( x, y )] = cost
		end
	end
	return n
end

local function getneighbors( level, nodeid )
	local x, y = id2xy( nodeid )
	local w, h = width, height
	local neighbors = {}
	addnodeif( x < w,           neighbors, level, x+1, y,   n )
	addnodeif( x > 1,           neighbors, level, x-1, y,   n )
	addnodeif( y < h,           neighbors, level, x,   y+1, n )
	addnodeif( y > 1,           neighbors, level, x,   y-1, n )
	addnodeif( x > 1 and y < h, neighbors, level, x-1, y+1, n, 1.41 )
	addnodeif( x > 1 and y > 1, neighbors, level, x-1, y-1, n, 1.41 )
	addnodeif( x < w and y < h, neighbors, level, x+1, y+1, n, 1.41 )
	addnodeif( x < w and y > 1, neighbors, level, x+1, y-1, n, 1.41 )
	return neighbors
end

local function getneighborsprealloc( level, nodeid, neighbors )
	local x, y = id2xy( nodeid )
	local w, h = width, height
	for k in next, neighbors do neighbors[k] = nil end
	addnodeif( x < w,           neighbors, level, x+1, y,   n )
	addnodeif( x > 1,           neighbors, level, x-1, y,   n )
	addnodeif( y < h,           neighbors, level, x,   y+1, n )
	addnodeif( y > 1,           neighbors, level, x,   y-1, n )
	addnodeif( x > 1 and y < h, neighbors, level, x-1, y+1, n, 1.41 )
	addnodeif( x > 1 and y > 1, neighbors, level, x-1, y-1, n, 1.41 )
	addnodeif( x < w and y < h, neighbors, level, x+1, y+1, n, 1.41 )
	addnodeif( x < w and y > 1, neighbors, level, x+1, y-1, n, 1.41 )
	return neighbors
end
local function heuristics( level, nodeid, goalid )
	local x0, y0 = id2xy( nodeid )
	local x1, y1 = id2xy( goalid )
	return ((x1-x0)^2 + (y1-y0)^2)^0.5
end

print( 'Walls are impassable' )
local level1 = makelevel( 1, math.huge )
local path = astar.find( getneighbors, heuristics, level1,
	xy2id( 1, height ), xy2id( width, 1 ))

for i = 1, #level1 do
	if level1[i] == math.huge then level1[i] = 9 end
end

for i = 1, #path do
	level1[path[i]] = 5
end

for y = 1, height do
	for x = 1, width do
		io.write( tostring(level1[xy2id(x, y)] ))
	end
	io.write('\n')
end
io.write('\n')


print( 'Walls are highly more coslty' )
local level2 = makelevel( 1, 100 )
local path = astar.find( getneighbors, heuristics, level2,
	xy2id( 1, height ), xy2id( width, 1 ))

for i = 1, #level2 do
	if level2[i] == 100 then level2[i] = 9 end
end

for i = 1, #path do
	level2[path[i]] = 5
end

for y = 1, height do
	for x = 1, width do
		io.write( tostring(level2[xy2id(x, y)] ))
	end
	io.write('\n')
end
io.write('\n')


print( 'Walls are slightly more coslty' )
local level3 = makelevel( 1, 2 )
local path = astar.find( getneighbors, heuristics, level3,
	xy2id( 1, height ), xy2id( width, 1 ))

for i = 1, #level3 do
	if level3[i] == 2 then level3[i] = 9 end
end

for i = 1, #path do
	level3[path[i]] = 5
end

for y = 1, height do
	for x = 1, width do
		io.write( tostring(level3[xy2id(x, y)] ))
	end
	io.write('\n')
end
io.write('\n')


print( 'No path' )
local level4 = makelevel( 1, math.huge )
level4[xy2id( 1, 2 )] = math.huge
level4[xy2id( width-1, height )] = math.huge
local path = astar.find( getneighbors, heuristics, level4,
	xy2id( 1, height ), xy2id( width, 1 ))

for i = 1, #level4 do
	if level4[i] == math.huge then level4[i] = 9 end
end

for y = 1, height do
	for x = 1, width do
		io.write( tostring(level4[xy2id(x, y)] ))
	end
	io.write('\n')
end
io.write('\n')

assert( path == nil )
