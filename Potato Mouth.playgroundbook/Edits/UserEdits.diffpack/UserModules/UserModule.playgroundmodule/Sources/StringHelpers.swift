
public extension StringProtocol {
    public func distance(of element: Element) -> Int? { firstIndex(of: element)?.distance(in: self) }
    public func distance<S: StringProtocol>(of string: S) -> Int? { range(of: string)?.lowerBound.distance(in: self) }
}

public extension Collection {
    public func distance(to index: Index) -> Int { distance(from: startIndex, to: index) }
}

public extension String.Index {
    public func distance<S: StringProtocol>(in string: S) -> Int { string.distance(to: self) }
}
