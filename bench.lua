local astar = require('astar')
local floor = math.floor

print( 'BENCHMARK' )
for _, width in ipairs{10, 20, 30} do
	local height = width
	local function id2xy( id )
		return ( (id-1) % width ) + 1, floor( (id-1) / width ) + 1
	end

	local function heuristics( level, nodeid, goalid )
		local x0, y0 = id2xy( nodeid )
		local x1, y1 = id2xy( goalid )
		return 10*((x1-x0)^2 + (y1-y0)^2)^0.5
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

	local function emptylevel( width, height, floorcost )
		local level = {}
		for i = 1, width*height do
			level[i] = floorcost
		end
		return level
	end

	local function randomlevel( width, height, floorcost, wallcost, walldensity )
		local level = {}
		for i = 1, width*height do
			level[i] = math.random() >= walldensity and floorcost or wallcost
		end
		return level
	end

	local function badlevel( width, height, floorcost, wallcost )
		local level = emptylevel( width, height, floorcost )
		for x = 2, width-1 do
			y = 2
			level[xy2id( x, y )] = wallcost
		end
		for y = 2, height-1 do
			x = width-1
			level[xy2id( x, y )] = wallcost
		end
		return level
	end
	local function getneighbors( level, nodeid )
		local x, y = id2xy( nodeid )
		local w, h = width, width
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
	local t0, n = os.clock(), 1e4
	for i = 1, n do
		local path = astar.find( getneighbors, heuristics, emptylevel( width, width, 1 ),
		xy2id( 1, width ), xy2id( width, 1 ))
	end
	print( n/(os.clock()-t0), ('%dx%d EMPTY'):format(width, width))

	for _, density in ipairs{0.1, 0.25, 0.5, 0.7} do
		local t0, n = os.clock(), 1e4
		for i = 1, n do
			local path = astar.find( getneighbors, heuristics, randomlevel( width, width, 1, 100, density ),
			xy2id( 1, width ), xy2id( width, 1 ))
		end
		print( n/(os.clock()-t0), ('%dx%d RANDOM %g'):format(width, width, density))
	end

	local t0, n = os.clock(), 1e4
	for i = 1, n do
		local path = astar.find( getneighbors, heuristics, badlevel( width, width, 1, 100 ),
		xy2id( 1, width ), xy2id( width, 1 ))
	end
	print( n/(os.clock()-t0), ('%dx%d BAD'):format(width, width))
end
