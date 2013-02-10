#region usings
using System;
using System.ComponentModel.Composition;
using System.Collections.Generic;
using System.Threading;

using VVVV.PluginInterfaces.V1;
using VVVV.PluginInterfaces.V2;
using VVVV.Utils.VColor;
using VVVV.Utils.VMath;

using VVVV.Core.Logging;
#endregion usings

namespace VVVV.Nodes
{
	#region PluginInfo
	[PluginInfo(Name = "Pathfinder", Category = "Value", Help = "Find shortest Path through a Grid with Obstacles", Tags = "AStar, velcrome, synopia")]
	#endregion PluginInfo
	public class ValuePathfinderNode : IPluginEvaluate, IDisposable
	{
		#region fields & pins
		[Config("Limit Size of SearchHeap", DefaultValue = 1, MinValue = 0.0, MaxValue = 1.0 )] 
		ISpread<double> FMaxSize; // Max Heap Size. 
		
		[Input("ObstacleMap", DefaultValue = 0.0)]
		ISpread<bool> FMapIn;
		
		[Input("MapSize XY", DefaultValue = 1.0)]
		ISpread<int> FMapSize;
		
		[Input("Update Map", IsBang = true)]
		ISpread<bool> FUpdate;
		
		[Input("Start")]
		IDiffSpread<Vector2D> FStart;
		
		[Input("Target")]
		IDiffSpread<Vector2D> FTarget;
		
		[Output("Path")]
		ISpread<ISpread<Vector2D>> FPathOut;
		
		[Output("PathValid")]
		ISpread<bool> FPathValid;
		
		
		[Import()]
		ILogger FLogger;
		
		PathfindingService FPathfindingService = null;
		#endregion fields & pins

		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
			SpreadMax = 0;
			SpreadMax = Math.Max(SpreadMax, FStart.SliceCount);
			SpreadMax = Math.Max(SpreadMax, FTarget.SliceCount);
			
			FPathValid.SliceCount = FPathOut.SliceCount = SpreadMax;
//			FLogger.Log(LogType.Debug, SpreadMax.ToString());
			
			bool mapHasChanged = false;
			if( FPathfindingService==null ) {
				FPathfindingService = new PathfindingService(FLogger);
				mapHasChanged = true;
			}
			
			if( FUpdate[0] || mapHasChanged ) {
				FPathfindingService.UpdateMap(FMapSize, FMapIn);
				mapHasChanged = true;
			}

			for (int i=0;i<SpreadMax;i++) {
				FPathfindingService.SetPath(i, FStart[i], FTarget[i], FMaxSize[i]);
			}
			FPathfindingService.Update(FPathOut, FPathValid);
		}
		
		public void Dispose() {			
			FPathfindingService.Stop();
		}
	}
	
	class PathfindingService {
		ReaderWriterLockSlim _rw = new ReaderWriterLockSlim();
		Pathfinder FPathFinder;
		int FSizeX;
		int FSizeY;
		IDictionary<Tuple<int,int>,Path> FPathes = new Dictionary<Tuple<int,int>,Path>();
		ILogger FLogger;
		
		IDictionary<int, Slot> FSlots = new Dictionary<int, Slot>();
		ISet<Path> FPathQueue = new HashSet<Path>();
		bool running;
		
		public PathfindingService(ILogger logger) {
			FLogger = logger;
			running = true;
			new Thread(ThreadLoop).Start();
		} 
		
		
		public void Stop() {
			running = false;
		}
		
		public void SetPath( int slotId, Vector2D start, Vector2D target, double maxHeapSize ) {
			Slot slot;
			if( FSlots.ContainsKey(slotId) ) {
				slot = FSlots[slotId];
			} else {
				slot = new Slot();
				FSlots.Add(slotId, slot);
			}
			int targetX = VMath.Zmod((int)target.x, FSizeX);  // Wrap X within Map
			int targetY = VMath.Zmod((int)target.y, FSizeY);  // Wrap Y within Map
			
			int startX = VMath.Zmod((int)start.x, FSizeX);
			int startY = VMath.Zmod((int)start.y, FSizeY);
			
			int startNode = Index( startX, startY );
			int targetNode = Index(targetX, targetY );
			var p = slot.PathId();
			bool update = slot.Update(startNode, targetNode, maxHeapSize);
			if( p!=null && update ) {
				_rw.EnterWriteLock();
				Path path = GetPath(p, maxHeapSize);
				if( FPathQueue.Contains(path) ) {
					FPathQueue.Remove(path);
				}
				_rw.ExitWriteLock();
			}
			
		}
		
		public void ThreadLoop() {
			
			FLogger.Log(LogType.Debug, "thread started");
			while( running ) {
				Thread.Sleep(1);
				_rw.EnterReadLock();
				var e = FPathQueue.GetEnumerator();
				Path path = null;
				if( e.MoveNext() ) {
					path = e.Current;
				}
				_rw.ExitReadLock();
				if( path==null ) {
					continue;
				}
//				Thread.Sleep(1000);
				path.FindPath( FPathFinder );

				_rw.EnterWriteLock();
				if( FPathQueue.Contains(path) ) {
					FPathQueue.Remove(path);
				}
				_rw.ExitWriteLock();
			}
			FLogger.Log(LogType.Debug, "thread ended");
		}
		
		public void Update(ISpread<ISpread<Vector2D>> result, ISpread<bool> valid) {
			foreach( int slotId in FSlots.Keys ) {
				Slot slot = FSlots[slotId];
				valid[slotId] = slot.IsRendered();
				if( !slot.IsRendered() ) {					
					Path path = GetPath( slot.PathId(), slot.maxHeapSize );
					IEnumerable<int> pathToDisplay = path.GetData();
					
					if( pathToDisplay!=null ) {
						result[slotId].SliceCount=0;						
						foreach( int step in pathToDisplay ) {
							result[slotId].Add(new Vector2D(step%FSizeX, step/FSizeX));
						}
						slot.SetRendered();
					} else {
						_rw.EnterWriteLock();
						if( !FPathQueue.Contains(path) ) {
							FPathQueue.Add(path);
							result[slotId].Add(new Vector2D(0,0));
						}
						_rw.ExitWriteLock();
					}					
				} 				
			}
		}
		
		private Path GetPath( Tuple<int,int> pathId, double maxHeapSize ) {
			Path path;
			if( FPathes.ContainsKey(pathId) ) {
				path = FPathes[pathId];
			} else {
				path = new Path(pathId.Item1, pathId.Item2);
				path.maxHeapSize = maxHeapSize;
				FPathes.Add(pathId, path);
			}			
			return path;
		}
		
		public void UpdateMap(ISpread<int> mapSize, ISpread<bool> map ) {
			FSizeX = mapSize[0];
			FSizeY = mapSize[1];
			FPathFinder = new Pathfinder(mapSize[0], mapSize[1], map);
			FPathFinder.SetLogger(FLogger);
			FPathes.Clear();
			foreach( Slot slot in FSlots.Values ) {
				slot.SetRendered(false);
			}
		}
		
		private int Index( int x, int y ) {
			return FPathFinder.Index(x,y);
		}
		private int Index( Vector2D pos ) {
			return Index( (int)pos.x, (int)pos.y );
		}
	}
	class Slot {
		private int startNode;
		private int targetNode;
		public double maxHeapSize{get;set;}
		private Tuple<int,int> pathId;		
		private bool rendered;
		
		public bool Update( int startNode, int targetNode, double maxHeapSize ) {
			if( startNode!=this.startNode || targetNode!=this.targetNode || maxHeapSize!=this.maxHeapSize ) {
				this.startNode = startNode;
				this.targetNode = targetNode;
				this.maxHeapSize = maxHeapSize;
				this.rendered = false;
				pathId = new Tuple<int,int>(startNode, targetNode);
				return true;
			} else {
				return false;
			}
		}
		public bool IsRendered() { return rendered; }
		public void SetRendered(bool v=true) { rendered = v; }
		public Tuple<int,int> PathId() { return pathId; }
	}
	class Path {
		private int startNode;
		private int targetNode;
		public double maxHeapSize{
			get;set;
		}
		private IEnumerable<int> path;
		
		public Path( int startNode, int targetNode ) {
			this.startNode = startNode;
			this.targetNode = targetNode;
		}
		
		public IEnumerable<int> GetData() { return path; }
		public bool HasData() {	return path!=null; }
		public void SetData( IEnumerable<int> path ) {this.path = path;}
		public void FindPath(Pathfinder finder){
			path = finder.FindPath( startNode, targetNode, maxHeapSize );
		}
	}
	
	class Pathfinder {
		int[] map;
		double[] gMap;
		double[] fMap;
		int[] ageMap;
		int[] predecessorMap;
		int FSizeX;
		int FSizeY;
		int[,] neighbours;
		
		BinaryHeap openList;
		ISet<int> closeList;
		
		int age;
		int targetX;
		int targetY;
		
		ILogger FLogger;
		
		public Pathfinder( int mapX, int mapY, ISpread<bool> Pmap ) {
			map = new int[mapY*mapX];
			this.FSizeX = mapX;
			this.FSizeY = mapY;

			int i=0;
			for( var y=0; y<mapY; y++ ) {
				for( var x=0; x<mapX; x++ ) {
					map[i] = Pmap[i] ? 1 : 0;
					i++;
				}
			}
			neighbours = new int[8,3]{
				{-1,-1, 2},{0,-1, 1},{1,-1, 2},
				{-1,0, 1},			 {1,0, 1},
				{-1,+1, 2},{0,+1, 1},{1,+1, 2}
			};
			ageMap = new int[FSizeX*FSizeY];
			gMap = new double[FSizeY*FSizeX];
			fMap = new double[FSizeY*FSizeX];
			predecessorMap = new int[FSizeY*FSizeX];
			openList = new BinaryHeap( delegate(int a, int b) {
				return fMap[a]<fMap[b] ? +1 : fMap[a]>fMap[b] ? -1 : 0;
			}, FSizeX*FSizeY, FSizeX*FSizeY);
			closeList = new HashSet<int>();
			predecessorMap = new int[FSizeY*FSizeX];
			age = 0;
		}
		
		public IEnumerable<int> FindPath( int startNode, int targetNode, double maxHeap = 1.0 ) {
			int maxCount = (int) Math.Floor(FSizeX * FSizeY * maxHeap);
			
			openList.Clear();
			closeList.Clear();
			age ++;
			
			int currentNode;
			
			openList.Add(startNode);
			bool found = false;
			gMap[openList.Min()]=0;
			ageMap[openList.Min()]=age;
			while( maxCount>0 && openList.GetSize()>0 ) {				
				currentNode = openList.RemoveMin();
				ageMap[currentNode]=age;
				if( currentNode==targetNode ) {
					found = true;					
					break;
				}
				expandNode( currentNode );
				closeList.Add(currentNode);
				maxCount --;
			}
			if( found ) {
				currentNode = targetNode;
				var path = new List<int>();
				int count = 0;
				while( currentNode!=0 && currentNode!=startNode ) {
					path.Add(currentNode);
					currentNode = predecessorMap[currentNode];
					if( ageMap[currentNode]!=age ) {
						currentNode = 0;
					}
					count ++;
				}
				path.Reverse();
				return path;
			} else {
				return null;
			}
		}
		
		private double cost( int fromNode, int toNode, int type ) {
			if( map[toNode]==1 ) {
				return Double.MaxValue;
			} else {
				return type==1 ? 1.0 : 1.41421;
			}
			
		}
		
		// manhattan distance
		private double heuristic( int node ) {
			int nodeX = node%FSizeX;
			int nodeY = node/FSizeX;
			
			//			return Math.Sqrt( (nodeX-targetX )*(nodeX-targetX ) + ( nodeY-targetY)*( nodeY-targetY));
			return Math.Abs( nodeX-targetX ) + Math.Abs( nodeY-targetY) ;
		}
		
		private void expandNode( int currentNode ) {
			int currentX = currentNode%FSizeX;
			int currentY = currentNode/FSizeY;
			for( int i=0; i<8; i++ ) {
				int x = neighbours[i,0]+currentX;
				int y = neighbours[i,1]+currentY;
				if( x<0 || y<0 || x>=FSizeX || y>=FSizeY ) {
					continue;
				}
				var neighbourNode = Index(x, y);
				if( map[neighbourNode]==1 ) {
					continue;
				}
				if( closeList.Contains(neighbourNode) ) {
					continue;
				}
				
				double tentative_g = gMap[currentNode] + cost( currentNode, neighbourNode, neighbours[i,2] );
				if( openList.Contains(neighbourNode) && ageMap[neighbourNode]==age && tentative_g>=gMap[neighbourNode] ) {
					continue;
				}
				
				predecessorMap[neighbourNode] = currentNode;
				gMap[neighbourNode] = tentative_g;
				fMap[neighbourNode] = tentative_g + heuristic( neighbourNode );
				ageMap[neighbourNode] = age;
				if( !openList.Contains(neighbourNode) ) {
					openList.Add(neighbourNode);
				} else {
					openList.Update(neighbourNode);
				}
			}
		}
		
		public int Index( int x, int y ) {
			return y*FSizeX+x;
		}
		
		public void SetLogger( ILogger logger ) {
			FLogger = logger;
		}
		
	}
	
	public delegate int Comparer(int a,int b);
	
	class BinaryHeap {
		public static int DEFAULT_CAPACITY = 1000;
		public static int DEFAULT_INDEX_SIZE = 1024;
		private int[] FHeap;
		private int[] FItemToIndex;
		private int FSize;
		private Comparer FComparator;
		
		public BinaryHeap(Comparer comparator, int capacity, int indexSize ) {
			this.FComparator = comparator;
			FSize = 0;
			FHeap = new int[capacity];
			FItemToIndex = new int[indexSize];
			Clear();
		}
		
		public int GetSize() {
			return FSize;
		}
		
		public bool IsEmpty() {
			return GetSize()==0;
		}
		
		public bool Contains( int item ) {
			return FItemToIndex[item]>-1;
		}
		
		public void Update( int item ) {
			ResortUp(FItemToIndex[item]);
		}
		
		public void Add( int item ) {
			if( FHeap.Length <= FSize ) {
				Enlarge( FHeap.Length * 2 );
			}
			
			int last = FSize;
			FSize ++;
			FHeap[last] = item;
			FItemToIndex[item] = last;
			ResortUp(last);
		}
		
		public int Min() {
			return FHeap[0];
		}
		
		public int RemoveMin() {
			int min = FHeap[0];
			FHeap[0] = FHeap[FSize-1];
			FItemToIndex[min] = -1;
			ResortDown(0);
			FSize--;
			return min;
		}
		
		private void ResortDown( int index ) {
			int val = FHeap[index];
			while( index*2+1<FSize ) {
				int child = index * 2 + 1;
				if( child<FSize-1 && FComparator(FHeap[child+1], FHeap[child])>0 ) {
					child ++;
				}
				if( FComparator(FHeap[child], val)>0 ) {
					FHeap[index] = FHeap[child];
					FItemToIndex[FHeap[child]] = index;
				} else {
					break;
				}
				index = child;
			}
			FHeap[index] = val;
			FItemToIndex[val] = index;
		}
		
		private void ResortUp(int index) {
			int val = FHeap[index];
			while( index>0 && FComparator(FHeap[index/2], val)<0 ) {
				FHeap[index] = FHeap[index/2];
				FItemToIndex[FHeap[index/2]] = index;
				index /= 2;
			}
			FHeap[index] = val;
			FItemToIndex[val] = index;
		}
		
		private void Enlarge( int newSize ) {
			int[] array = new int[newSize];
			Array.Copy(FHeap, array, FSize );
			FHeap = array;
			
		}
		
		public void Clear() {
			FSize = 0;
			for( int i=0; i<FItemToIndex.Length; i++ ) {
				FItemToIndex[i]=-1;
			}
		}
	}
}
