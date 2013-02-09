#region usings
using System;
using System.ComponentModel.Composition;
using System.Collections.Generic;

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
	public class ValuePathfinderNode : IPluginEvaluate
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
		
		[Import()]
		ILogger FLogger;
		
		Pathfinder FPathFinder;
		
		#endregion fields & pins

		
		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
			SpreadMax = (int)Math.Ceiling( FMapSize.SliceCount/2.0);
			SpreadMax = Math.Max(SpreadMax, FUpdate.SliceCount);
			SpreadMax = Math.Max(SpreadMax, FStart.SliceCount);
			SpreadMax = Math.Max(SpreadMax, FTarget.SliceCount);
			
			FPathOut.SliceCount = SpreadMax;
//			FLogger.Log(LogType.Debug, SpreadMax.ToString());
			bool mapHasChanged = false;
			if( FUpdate[0] || FPathFinder==null ) {
				FPathFinder = new Pathfinder(FMapSize[0], FMapSize[1], FMapIn);
				FPathFinder.SetLogger(FLogger);
				mapHasChanged = true;
			}

			for (int i=0;i<SpreadMax;i++) {
				if (FStart.IsChanged || FTarget.IsChanged || mapHasChanged) {
					IEnumerable<Vector2D> pathToDisplay = null;
					pathToDisplay = FPathFinder.FindPath( FStart[i], FTarget[i], FMaxSize[0] );
					FPathOut[i].SliceCount=0;
					if( pathToDisplay!=null ) {
						foreach( Vector2D step in pathToDisplay ) {
							FPathOut[i].Add(step);
						}
						
					}
				}
			}
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
		
		public IEnumerable<Vector2D> FindPath( Vector2D start, Vector2D target, double maxHeap = 1.0 ) {
			int maxCount = (int) Math.Floor(FSizeX * FSizeY * maxHeap);
			int origMaxCount = maxCount;
			
			targetX = VMath.Zmod((int)target.x, FSizeX);  // Wrap X within Map
			targetY = VMath.Zmod((int)target.y, FSizeY);  // Wrap Y within Map
			
			for( int i=0;i<predecessorMap.Length; i++) {
				predecessorMap[i]=0;
			}
			openList.Clear();
			closeList.Clear();
			age ++;
			
			int currentNode;
			int targetNode = Index(target);
			openList.Add(Index(start));
			bool found = false;
			gMap[openList.Min()]=0;
			ageMap[openList.Min()]=age;
			while( maxCount>0 && openList.GetSize()>0 ) {				
				currentNode = openList.RemoveMin();
				if( currentNode==targetNode ) {
					found = true;
					break;
				}
				expandNode( currentNode );
				closeList.Add(currentNode);
				maxCount --;
			}
			FLogger.Log(LogType.Debug, "found= "+found+" "+(origMaxCount-maxCount));
			if( found ) {
				currentNode = targetNode;
				var path = new List<Vector2D>();
				while( currentNode!=0 ) {
					path.Add(new Vector2D(currentNode%FSizeX, currentNode/FSizeY));
					currentNode = predecessorMap[currentNode];
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
		
		private int Index( int x, int y ) {
			return y*FSizeX+x;
		}
		private int Index( Vector2D pos ) {
			return Index( (int)pos.x, (int)pos.y );
		}
		
		public void SetLogger( ILogger logger ) {
			FLogger = logger;
		}
		
		public double GetG( Vector2D pos ) {
			return gMap[Index(pos)];
		}
		public double GetF( Vector2D pos ) {
			return fMap[Index(pos)];
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
