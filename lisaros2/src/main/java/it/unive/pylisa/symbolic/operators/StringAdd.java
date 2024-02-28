package it.unive.pylisa.symbolic.operators;

import it.unive.lisa.symbolic.value.operator.binary.StringConcat;

public class StringAdd extends StringConcat {

	/**
	 * The singleton instance of this class.
	 */
	public static final StringAdd INSTANCE = new StringAdd();

	/**
	 * Builds the operator. This constructor is visible to allow subclassing:
	 * instances of this class should be unique, and the singleton can be
	 * retrieved through field {@link #INSTANCE}.
	 */
	protected StringAdd() {
	}

	@Override
	public String toString() {
		return "+";
	}
}