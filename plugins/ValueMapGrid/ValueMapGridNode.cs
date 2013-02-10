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
	
	public class Grid {
		public int Count {get;set;}
		public double Min{get;set;}
		public double Max{get;set;}
		public int Order {get;set;}
		
		public Grid() {
			Min = 0;
			Max = 1;
			Count = 1;
			Order = 0;
			
		}
		public Grid(double min, double max, int count, int order) {
			Min = min;
			Max = max;
			Count = count;
			Order = order;
		}
		
		public Grid(Vector3D initSeed) {
			Min = initSeed.x;
			Max = initSeed.y;
			Count = (int)initSeed.z;
		}
		
		public double GetValue(int index) {
			return VMath.Map(index, 0, Count-1, Min, Max, TMapMode.Clamp);
		}
		
		
		public int GetIndex(double val) {
			return (int)VMath.Map(val, Min, Max, 0, Count, TMapMode.Clamp);
		}
	}
	
	#region PluginInfo
	[PluginInfo(Name = "MapGrid", Category = "Grid", Help = "Create a handy Grid", Tags = "velcrome, Spread")]
	#endregion PluginInfo
	public class ValueMapGridNode : IPluginEvaluate
	{
		#region fields & pins
		[Input("Source Minimum", DefaultValue = 0.0)]
		IDiffSpread<double> FMin;
		
		[Input("Source Maximum", DefaultValue = 1.0)]
		IDiffSpread<double> FMax;
		
		[Input("Vector Size", DefaultValue = 1.0, IsSingle = true, MinValue = 1.0)]
		IDiffSpread<int> FVectorSize;
		
		[Input("Count", DefaultValue = 1.0)]
		IDiffSpread<int> FCount;
		
		[Output("Grid")]
		ISpread<Grid> FGridOut;
		
		[Output("Empty Grid")]
		ISpread<ISpread<double>> FEmptyGridOut;
		
		
		[Import()]
		ILogger FLogger;
		#endregion fields & pins
		
		//called when data for any output pin is requested

		
		public void Evaluate(int SpreadMax)
		{
			if (!FMax.IsChanged && !FMin.IsChanged && !FVectorSize.IsChanged && !FCount.IsChanged) return;
			
			double vSize = FVectorSize[0];
			
			SpreadMax = FCount.SliceCount;
			SpreadMax = Math.Max(SpreadMax, FMin.SliceCount);
			SpreadMax = Math.Max(SpreadMax, FMax.SliceCount);
			
			SpreadMax = (int)Math.Ceiling(SpreadMax/(double)FVectorSize[0])*FVectorSize[0];
			
			FEmptyGridOut.SliceCount = SpreadMax;
			FGridOut.SliceCount = 0;
			
			
			for (int i = 0; i < SpreadMax; i++) {
				FEmptyGridOut[i].SliceCount = (int)VMath.Pow(FCount[i], FVectorSize[0]);
				
				int order = VMath.Zmod(i, FVectorSize[0]);
				//				for (int v=0;v<FVectorSize[0];v++)
				FGridOut.Add(new Grid(FMin[i], FMax[i], FCount[i], order));
				
			}
			//FLogger.Log(LogType.Debug, "hi tty!");
		}
	}
	
	#region PluginInfo
	[PluginInfo(Name = "MapGrid", Category = "Value", Help = "Create a handy Grid", Tags = "velcrome, Spread")]
	#endregion PluginInfo
	public class MapGridNode : IPluginEvaluate
	{
		#region fields & pins
		[Input("Input", DefaultValue = 0.0)]
		IDiffSpread<double> FInput;
		
		[Input("Grid")]
		ISpread<Grid> FGrid;
		
		[Output("Index")]
		ISpread<int> FIndex;
		
		
		[Import()]
		ILogger FLogger;
		#endregion fields & pins
		
		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
			if (FGrid.SliceCount >= 1 && FGrid[0] == null) FGrid[0] = new Grid();
			SpreadMax = FInput.SliceCount;
			
			
			//			if (!FInput.IsChanged) return;
			
			FIndex.SliceCount = SpreadMax;
			for (int i=0;i<SpreadMax;i++) {
				FIndex[i] = FGrid[i].GetIndex(FInput[i]);
				
			}
			
			//FLogger.Log(LogType.Debug, "hi tty!");
		}
	}
	
	#region PluginInfo
	[PluginInfo(Name = "MapGrid", Category = "Integer", Help = "Create a handy Grid", Tags = "velcrome, Spread")]
	#endregion PluginInfo
	public class IntMapGridNode : IPluginEvaluate
	{
		
		#region fields & pins
		[Input("Grid")]
		ISpread<Grid> FGrid;
		
		[Input("Index", DefaultValue = 0.0)]
		IDiffSpread<int> FIndex;
		
		
		[Output("Output")]
		ISpread<double> FOutput;
		
		[Import()]
		ILogger FLogger;
		#endregion fields & pins
		
		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
			if (FGrid.SliceCount >= 1 && FGrid[0] == null) FGrid[0] = new Grid();
			SpreadMax = FIndex.SliceCount;
			
			//			if (!FIndex.IsChanged) return;
			
			FOutput.SliceCount = SpreadMax;
			for (int i=0;i<SpreadMax;i++) {
				FOutput[i] = FGrid[i].GetValue(FIndex[i]);
				
			}
			
			//FLogger.Log(LogType.Debug, "hi tty!");
		}
	}
	
	#region PluginInfo
	[PluginInfo(Name = "GetSlice", Category = "Grid", Help = "Create a handy Grid", Tags = "velcrome, Spread")]
	#endregion PluginInfo
	public class GetSliceGridNode : IPluginEvaluate
	{
		
		#region fields & pins
		[Input("Grid")]
		ISpread<Grid> FGrid;
		
		[Input("Index", DefaultValue = 0.0)]
		IDiffSpread<int> FIndex;
		
		[Output("Grid")]
		ISpread<Grid> FOutput;
		
		[Output("Vector Size")]
		ISpread<int> FVectorSize;
		
		[Import()]
		ILogger FLogger;
		#endregion fields & pins
		
		//called when data for any output pin is requested
		public void Evaluate(int SpreadMax)
		{
			if (FGrid.SliceCount >= 1 && FGrid[0] == null) FGrid[0] = new Grid();
			SpreadMax = FIndex.SliceCount;
			
			//			if (!FIndex.IsChanged) return;
			
			FOutput.SliceCount = 0;
			
			int count = -1;
			List<List<Grid>> list = new List<List<Grid>>();
			foreach (Grid g in FGrid) {
				if (g.Order == 0) {
					count++;
					list.Add(new List<Grid>());
				}
				list[count].Add(g);
			}
			
			
			for (int i=0;i<SpreadMax;i++) {
				FOutput.AddRange( list[VMath.Zmod(FIndex[i], count+1)] );
				
			}
			
			//FLogger.Log(LogType.Debug, "hi tty!");
		}
	}
}
