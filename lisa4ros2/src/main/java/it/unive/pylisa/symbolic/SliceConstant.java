package it.unive.pylisa.symbolic;

import it.unive.lisa.analysis.numeric.Interval;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.util.numeric.MathNumber;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.analysis.dataframes.operations.selection.SliceElement;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.Optional;

public class SliceConstant extends Constant {

	public SliceConstant(
			Slice slice,
			CodeLocation location) {
		super(PyClassType.lookup(LibrarySpecificationProvider.SLICE), slice, location);
	}

	public SliceConstant(
			RangeBound start,
			RangeBound end,
			RangeBound skip,
			CodeLocation location) {
		super(PyClassType.lookup(LibrarySpecificationProvider.SLICE), new Slice(start, end, skip), location);
	}

	public static class RangeBound implements SliceElement {

		private final Optional<Integer> bound;

		public RangeBound() {
			this.bound = Optional.empty();
		}

		public RangeBound(
				int value) {
			this.bound = Optional.of(value);
		}

		public Interval toInterval() {
			if (bound.isPresent())
				return new Interval(bound.get(), bound.get());
			else
				return new Interval(MathNumber.ZERO, MathNumber.PLUS_INFINITY);
		}

		public ConstantPropagation toConstant() {
			if (bound.isPresent())
				return new ConstantPropagation(bound.get());
			else
				return new ConstantPropagation().top();
		}

		@Override
		public String toString() {
			return toConstant().toString();
		}

		@Override
		public int hashCode() {
			final int prime = 31;
			int result = 1;
			result = prime * result + ((bound == null) ? 0 : bound.hashCode());
			return result;
		}

		@Override
		public boolean equals(
				Object obj) {
			if (this == obj)
				return true;
			if (obj == null)
				return false;
			if (getClass() != obj.getClass())
				return false;
			RangeBound other = (RangeBound) obj;
			if (bound == null) {
				if (other.bound != null)
					return false;
			} else if (!bound.equals(other.bound))
				return false;
			return true;
		}

		@Override
		public ConstantPropagation getEndIndex() {
			if (bound.isEmpty())
				return new ConstantPropagation().top();
			return new ConstantPropagation(bound.get());
		}

		@Override
		public ConstantPropagation getBeginIndex() {
			return getEndIndex();
		}
	}

	public static class Slice {
		private RangeBound start, end, skip;

		public Slice(
				RangeBound start,
				RangeBound end,
				RangeBound skip) {
			this.start = start;
			this.end = end;
			this.skip = skip;
		}

		public RangeBound getStart() {
			return start;
		}

		public RangeBound getEnd() {
			return end;
		}

		public RangeBound getSkip() {
			return skip;
		}

		@Override
		public boolean equals(
				Object obj) {
			if (!(obj instanceof Slice))
				return false;
			Slice o = (Slice) obj;
			if (this.start == null) {
				if (o.start != null)
					return false;
			} else if (!this.start.equals(o.start))
				return false;

			if (this.end == null) {
				if (o.end != null)
					return false;
			} else if (!this.end.equals(o.end))
				return false;

			if (this.skip == null) {
				if (o.skip != null)
					return false;
			} else if (!this.skip.equals(o.skip))
				return false;
			return true;
		}

		@Override
		public String toString() {
			return start + ":" + end + ":" + skip;
		}
	}
}
