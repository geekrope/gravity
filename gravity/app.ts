interface IParameters
{
	acceleration: DOMPoint;
	velocity: DOMPoint;
	baseVelocity: DOMPoint;
	movement: DOMPoint;
	fullVelocity: DOMPoint;
}

class Utils
{
	public static distance(point1: DOMPoint, point2: DOMPoint): number
	{
		return Math.sqrt((point1.x - point2.x) * (point1.x - point2.x) + (point1.y - point2.y) * (point1.y - point2.y));
	}
	public static trapezeArea(base1: number, base2: number, height: number): number
	{
		return (base1 + base2) * height / 2;
	}
	public static addVectors(...vectors: DOMPoint[]): DOMPoint
	{
		let result = new DOMPoint();

		vectors.forEach((vector) =>
		{
			result.x += vector.x;
			result.y += vector.y;
		});

		return result;
	}
	public static substractVectors(vector1: DOMPoint, vector2: DOMPoint): DOMPoint
	{
		return new DOMPoint(vector1.x - vector2.x, vector1.y - vector2.y);
	}
	public static multiplyVectors(vector1: DOMPoint, vector2: DOMPoint): number
	{
		return vector1.x * vector2.x + vector1.y * vector2.y;
	}
	public static multiplyVectorByScalar(vector: DOMPoint, scalar: number): DOMPoint
	{
		return new DOMPoint(vector.x * scalar, vector.y * scalar);
	}
}

class BodyParameters implements IParameters
{
	public acceleration: DOMPoint;
	public velocity: DOMPoint;
	public baseVelocity: DOMPoint;
	public movement: DOMPoint;

	public get fullVelocity(): DOMPoint
	{
		return Utils.addVectors(this.baseVelocity, this.velocity);
	}

	public constructor(acceleration: DOMPoint = new DOMPoint(), velocity: DOMPoint = new DOMPoint(), baseVelocity: DOMPoint = new DOMPoint(), movement: DOMPoint = new DOMPoint())
	{
		this.acceleration = acceleration;
		this.velocity = velocity;
		this.baseVelocity = baseVelocity;
		this.movement = movement;
	}
}

class ReadonlyBodyParameters implements IParameters
{
	public readonly acceleration: DOMPoint;
	public readonly velocity: DOMPoint;
	public readonly baseVelocity: DOMPoint;
	public readonly movement: DOMPoint;

	public get fullVelocity(): DOMPoint
	{
		return Utils.addVectors(this.baseVelocity, this.velocity);
	}

	public constructor(acceleration: DOMPoint = new DOMPoint(), velocity: DOMPoint = new DOMPoint(), baseVelocity: DOMPoint = new DOMPoint(), movement: DOMPoint = new DOMPoint())
	{
		this.acceleration = acceleration;
		this.velocity = velocity;
		this.baseVelocity = baseVelocity;
		this.movement = movement;
	}
}

class ObservableCollection<T>
{
	private _collection: T[];
	private _onChanged?: () => void;

	public get onChanged(): (() => void) | undefined
	{
		return this._onChanged;
	}
	public get collection(): ReadonlyArray<T>
	{
		return this._collection as ReadonlyArray<T>;
	}

	public set onChanged(value: (() => void) | undefined)
	{
		this._onChanged = value;
	}

	private invokeChangedEvent(): void
	{
		if (this._onChanged)
		{
			this._onChanged();
		}
	}

	public add(value: T)
	{
		this._collection.push(value);
		this.invokeChangedEvent();
	}
	public delete(index: number)
	{
		this._collection.splice(index, 1);
		this.invokeChangedEvent();
	}

	constructor()
	{
		this._collection = [];
	}
}

class PhysicalBody
{
	private _force: DOMPoint;
	private _position: DOMPoint;
	private _path: ObservableCollection<DOMPoint>;
	private _mass: number;
	private _parameters: BodyParameters;

	private static readonly maxPathLength = 255;

	public get force(): DOMPointReadOnly
	{
		return this._force as DOMPointReadOnly;
	}
	public get position(): DOMPointReadOnly
	{
		return this._position as DOMPointReadOnly;
	}
	public get mass(): number
	{
		return this._mass;
	}
	public get parameters(): ReadonlyBodyParameters
	{
		return this._parameters as ReadonlyBodyParameters;
	}
	public get path(): ReadonlyArray<DOMPoint>
	{
		return this._path.collection as ReadonlyArray<DOMPoint>;
	}

	public set position(position: DOMPointReadOnly)
	{
		this._path.add(this._position);
		this._position = position;
	}
	public set mass(mass: number)
	{
		this._mass = mass;
	}
	public set parameters(parameters: ReadonlyBodyParameters)
	{
		this._parameters = parameters;
	}

	private reducePath(): void
	{
		if (this._path.collection.length > PhysicalBody.maxPathLength)
		{
			this._path.delete(0);
		}
	}

	public addForce(force: DOMPointReadOnly): void
	{
		this._force = Utils.addVectors(this._force, force);
	}
	public setForce(force: DOMPointReadOnly): void
	{
		this._force = force;
	}
	public clearForces()
	{
		this._force = new DOMPoint();
	}

	public constructor(position: DOMPoint, mass: number, parameters: BodyParameters)
	{
		this._force = new DOMPoint();
		this._parameters = parameters;
		this._path = new ObservableCollection();
		this._position = position;
		this._mass = mass;

		this._path.onChanged = this.reducePath.bind(this);
	}
}

class PhysicalEngine
{
	private readonly G: number = 6.68e-11;
	private readonly maxTimeDelta: number = 10;

	private _bodyes: PhysicalBody[];
	private _timeOffset: number;
	private _speed: number;

	public static readonly timeUnit = 1000;
	public static readonly weightUnit = 1e15;

	public get speed(): number
	{
		return this._speed;
	}
	public get bodyies(): PhysicalBody[]
	{
		return this._bodyes;
	}

	public set speed(value: number)
	{
		this._speed = value;
	}

	private exists(body: PhysicalBody): boolean
	{
		return this._bodyes.indexOf(body) != -1;
	}
	private getBodyIndex(body: PhysicalBody): number
	{
		return this._bodyes.indexOf(body);
	}
	private computeForce(influencedBody: PhysicalBody): void
	{
		const formula = (mass1: number, mass2: number, distance: number) =>
		{
			return this.G * mass1 * mass2 / (distance * distance);
		}

		const influencedBodyIndex = this.getBodyIndex(influencedBody);

		if (influencedBodyIndex != -1)
		{
			influencedBody.clearForces();

			for (let influencingBodyIndex = 0; influencingBodyIndex < this._bodyes.length; influencingBodyIndex++)
			{
				if (influencingBodyIndex != influencedBodyIndex)
				{
					const influencingBody = this._bodyes[influencingBodyIndex];
					const forceValue = formula(influencedBody.mass, influencingBody.mass, Utils.distance(influencedBody.position, influencingBody.position));
					const angle = Math.atan2(influencingBody.position.y - influencedBody.position.y, influencingBody.position.x - influencedBody.position.x);
					const forceVector = new DOMPoint(forceValue * Math.cos(angle), forceValue * Math.sin(angle));

					influencedBody.addForce(forceVector);
				}
			}
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeAcceleration(body: PhysicalBody): DOMPoint
	{
		if (this.exists(body))
		{
			// F = ma
			return Utils.multiplyVectorByScalar(body.force, 1 / body.mass);
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeVelocity(body: PhysicalBody, timeDelta: number, acceleration: DOMPoint): DOMPoint
	{
		if (this.exists(body))
		{
			// finds area (difference between antiderivatives) between current acceleration and last captured acceleration
			return new DOMPoint(Utils.trapezeArea(body.parameters.acceleration.x, acceleration.x, timeDelta), Utils.trapezeArea(body.parameters.acceleration.y, acceleration.y, timeDelta)); // first integral of acceleration
		}
		else
		{
			throw new Error("Can't find body");
		}
	}
	private computeMovement(body: PhysicalBody, timeDelta: number, velocity: DOMPoint): DOMPoint
	{
		if (this.exists(body))
		{
			// finds area (difference between antiderivatives) between current velocity and last captured velocity
			return new DOMPoint(Utils.trapezeArea(body.parameters.velocity.x, velocity.x, timeDelta), Utils.trapezeArea(body.parameters.velocity.y, velocity.y, timeDelta));
		}
		else
		{
			throw new Error("Can't find body");
		}
	}

	public update()
	{
		const timeDelta = (Date.now() - this._timeOffset) * this._speed;

		this._bodyes.forEach((body) =>
		{
			for (let chunk = 0; chunk < Math.ceil(timeDelta / this.maxTimeDelta); chunk++)
			{
				const computedTimeDelta = Math.min(this.maxTimeDelta, timeDelta - chunk * this.maxTimeDelta);

				this.computeForce(body);
				const acceleration = this.computeAcceleration(body);
				const velocity = this.computeVelocity(body, computedTimeDelta / PhysicalEngine.timeUnit, acceleration);
				const movement = this.computeMovement(body, computedTimeDelta / PhysicalEngine.timeUnit, Utils.addVectors(velocity, body.parameters.baseVelocity));

				body.position = Utils.addVectors(movement, body.position);
				// area addition rule (S = S1 + S2);
				body.parameters = new BodyParameters(
					acceleration,
					Utils.addVectors(velocity, body.parameters.velocity),
					body.parameters.baseVelocity,
					Utils.addVectors(movement, body.parameters.movement)
				);
			}
		});

		this._timeOffset = Date.now();
	}
	public resetTimeOffset()
	{
		this._timeOffset = Date.now();
	}

	public constructor(bodyes: PhysicalBody[])
	{
		this._bodyes = bodyes;
		this._timeOffset = Date.now();
		this._speed = 1;
	}
}

class VisualEngine
{
	private readonly _emptyMatrix = new DOMMatrixReadOnly([1, 0, 0, 1, 0, 0]);
	private _context: CanvasRenderingContext2D;
	private _transform: DOMMatrixReadOnly;
	private _renderTransform: DOMMatrixReadOnly;
	private _offset: DOMPoint;
	private _planet: HTMLImageElement;

	public get context(): CanvasRenderingContext2D
	{
		return this._context;
	}
	public get offset(): DOMPointReadOnly
	{
		return this._offset as DOMPointReadOnly;
	}
	public get renderTransform(): DOMMatrixReadOnly
	{
		return this._renderTransform;
	}

	public set transform(value: DOMMatrixReadOnly)
	{
		this._transform = value;
		this._renderTransform = this.evaluateRenderTransform();
	}
	public set offset(value: DOMPointReadOnly)
	{
		this._offset = value;
		this._renderTransform = this.evaluateRenderTransform();
	}

	private evaluateRenderTransform(): DOMMatrixReadOnly
	{
		return this._transform.translate(this._offset.x, this._offset.y);
	}
	private applyTransform(): void
	{
		this._context.setTransform(this._renderTransform);
	}
	private drawBodySprite(body: PhysicalBody)
	{
		this._context.save();
		this.applyTransform();

		const size = Math.log(body.mass / PhysicalEngine.weightUnit + Math.E) / Math.LOG2E * 50;
		const width = size;
		const height = this._planet.height / this._planet.width * width;

		this.context.drawImage(this._planet, body.position.x - width / 2, body.position.y - height / 2, width, height);

		this._context.restore();
	}

	public drawBody(body: PhysicalBody)
	{
		if (this._planet.complete)
		{
			this.drawBodySprite(body);
		}
		else
		{
			this._planet.onload = () =>
			{
				this.drawBodySprite(body);
			}
		}
	}
	public drawVelocity(body: PhysicalBody)
	{
		this._context.save();
		this.applyTransform();

		const vectorStart = body.position;
		const vectorEnd = Utils.addVectors(body.position, body.parameters.fullVelocity);
		const arrowSize = 10;
		const arrowAngle = Math.PI / 4;
		const vectorAngle = Math.atan2(vectorEnd.y - vectorStart.y, vectorEnd.x - vectorStart.x);
		const arrowEnd_1 = new DOMPoint(vectorEnd.x - Math.cos(vectorAngle - arrowAngle) * arrowSize, vectorEnd.y - Math.sin(vectorAngle - arrowAngle) * arrowSize);
		const arrowEnd_2 = new DOMPoint(vectorEnd.x - Math.cos(vectorAngle + arrowAngle) * arrowSize, vectorEnd.y - Math.sin(vectorAngle + arrowAngle) * arrowSize);

		this._context.strokeStyle = "lime";

		this._context.beginPath();

		this._context.moveTo(vectorStart.x, vectorStart.y);
		this._context.lineTo(vectorEnd.x, vectorEnd.y);
		this._context.lineTo(arrowEnd_1.x, arrowEnd_1.y);
		this._context.moveTo(vectorEnd.x, vectorEnd.y);
		this._context.lineTo(arrowEnd_2.x, arrowEnd_2.y);

		this._context.stroke();

		this._context.restore();
	}
	public drawPath(body: PhysicalBody)
	{
		this._context.save();

		if (body.path.length > 0)
		{
			this.applyTransform();
			this._context.strokeStyle = "gray";

			for (let index = 1; index < body.path.length; index++)
			{
				this._context.beginPath();

				this._context.globalAlpha = index / body.path.length;

				this._context.moveTo(body.path[index - 1].x, body.path[index - 1].y);
				this._context.lineTo(body.path[index].x, body.path[index].y);

				this._context.stroke();
			}
		}

		this._context.restore();
	}
	public drawFps(fpsCounter: FpsCounter)
	{
		this._context.save();

		const fontSize = 16;
		const margin = 8;

		this._context.setTransform(this._emptyMatrix);
		this._context.fillStyle = "white";
		this._context.font = `${fontSize}px serif`;
		this._context.textBaseline = "top";
		this._context.textAlign = "left";

		this._context.fillText(fpsCounter.fps.toString(), margin, margin);

		this._context.restore();
	}

	public constructor(context: CanvasRenderingContext2D, offset: DOMPoint, bodySprite: string)
	{
		this._context = context;
		this._offset = offset;
		this._transform = this._emptyMatrix;
		this._renderTransform = this.evaluateRenderTransform();
		this._planet = new Image();

		this._planet.src = bodySprite;
	}
}

class Oscillator
{
	private _volume: number;
	private _frequency: number;
	private _oscillatorType: OscillatorType;
	private _audioContext: AudioContext;
	private _oscillatorNode: OscillatorNode;
	private _gainNode: GainNode;

	public get volume(): number
	{
		return this._volume;
	}
	public get frequency(): number
	{
		return this._frequency;
	}
	public get oscillatorType(): OscillatorType
	{
		return this._oscillatorType;
	}
	public get audioContext(): AudioContext
	{
		return this._audioContext;
	}
	public get oscillatorNode(): OscillatorNode
	{
		return this._oscillatorNode;
	}
	public get gainNode(): GainNode
	{
		return this._gainNode;
	}

	public set volume(value: number)
	{
		this._volume = value;
		this._gainNode.gain.value = value;
	}
	public set frequency(value: number)
	{
		this._frequency = value;
		this._oscillatorNode.frequency.value = value;
	}
	public set oscillatorType(value: OscillatorType)
	{
		this._oscillatorType = value;
		this._oscillatorNode.type = value;
	}

	public start(): void
	{
		this._oscillatorNode.start();
	}
	public stop(): void
	{
		this._oscillatorNode.stop();
	}

	public constructor(volume: number, frequency: number, type: OscillatorType)
	{
		this._volume = volume;
		this._frequency = frequency;
		this._oscillatorType = type;

		this._audioContext = new AudioContext();
		this._oscillatorNode = new OscillatorNode(this._audioContext);
		this._gainNode = new GainNode(this._audioContext);

		this._oscillatorNode.type = this._oscillatorType;
		this._oscillatorNode.frequency.value = this._frequency;
		this._gainNode.gain.value = this._volume;

		this._oscillatorNode.connect(this._gainNode);
		this._gainNode.connect(this._audioContext.destination);
	}
}

class FpsCounter
{
	private _timeOffset: number
	private _frames: number;
	private _fps: number;

	public get fps(): number
	{
		return this._fps;
	}

	public tick()
	{
		if (Date.now() - this._timeOffset > 1000)
		{
			this._fps = this._frames;
			this._frames = 0;
			this._timeOffset = Date.now();
		}

		this._frames++;
	}

	public constructor()
	{
		this._timeOffset = Date.now();
		this._frames = 0;
		this._fps = 0;
	}
}

type playgroundEvents = "update";

class Playground
{
	private _editedBody?: PhysicalBody;
	private _onUpdate: ((playground: this) => void)[];
	private readonly _canvas: HTMLCanvasElement;
	private readonly _context: CanvasRenderingContext2D;
	private readonly _physicalEngine: PhysicalEngine;
	private readonly _visualEngine: VisualEngine;
	private readonly _fpsCounter: FpsCounter;
	private _oscillator?: Oscillator;

	public get canvas(): HTMLCanvasElement
	{
		return this._canvas;
	}
	public get context(): CanvasRenderingContext2D
	{
		return this._context;
	}
	public get physicalEngine(): PhysicalEngine
	{
		return this._physicalEngine;
	}
	public get visualEngine(): VisualEngine
	{
		return this._visualEngine;
	}
	public get fpsCounter(): FpsCounter
	{
		return this._fpsCounter;
	}
	public get oscillator(): Oscillator | undefined
	{
		return this._oscillator;
	}
	public get editedBody(): PhysicalBody | undefined
	{
		return this._editedBody;
	}

	public set oscillator(value: Oscillator | undefined)
	{
		this._oscillator = value;
	}
	public set editedBody(value: PhysicalBody | undefined)
	{
		this._editedBody = value;
	}

	public addEventListener(type: "update", handler: (playground: this) => void): void
	public addEventListener(type: playgroundEvents, handler: (playground: this) => void)
	{
		switch (type)
		{
			case "update":
				this._onUpdate.push(handler);
		}
	}

	public removeEventListener(type: "update", handler: (playground: this) => void): void
	public removeEventListener(type: playgroundEvents, handler: (playground: this) => void)
	{
		switch (type)
		{
			case "update":
				this._onUpdate.splice(this._onUpdate.indexOf(handler));
		}
	}

	public constructor(canvas: HTMLCanvasElement, context: CanvasRenderingContext2D, physicalEngine: PhysicalEngine, visualEngine: VisualEngine)
	{
		this._canvas = canvas;
		this._context = context;
		this._physicalEngine = physicalEngine;
		this._visualEngine = visualEngine;
		this._fpsCounter = new FpsCounter();
		this._onUpdate = [];

		setInterval(() =>
		{
			this._onUpdate.forEach((handler) =>
			{
				handler(this);
			});
		}, 10);
	}
}

function draw(playground: Playground)
{
	playground.physicalEngine.update();
	playground.context.fillStyle = "black";
	playground.context.fillRect(0, 0, playground.canvas.width, playground.canvas.height);

	playground.physicalEngine.bodyies.forEach((body) =>
	{
		playground.visualEngine.drawBody(body);
		playground.visualEngine.drawVelocity(body);
		playground.visualEngine.drawPath(body);
	});

	if (playground.editedBody)
	{
		playground.visualEngine.drawBody(playground.editedBody);
		playground.visualEngine.drawPath(playground.editedBody);
	}

	playground.visualEngine.drawFps(playground.fpsCounter);
}
function addBodyMass(playground: Playground)
{
	if (playground.editedBody)
	{
		playground.editedBody.mass += PhysicalEngine.weightUnit / 50;

		if (playground.oscillator)
		{
			playground.oscillator.frequency = Math.min(1000, playground.oscillator.frequency + 1);
		}
	}
}
function mouseDownHandler(playground: Playground): (event: MouseEvent) => void
{
	return (function (this: typeof playground, event: MouseEvent)
	{
		this.editedBody = new PhysicalBody(new DOMPoint(event.offsetX - playground.visualEngine.offset.x, event.offsetY - playground.visualEngine.offset.y), PhysicalEngine.weightUnit, new BodyParameters());
		this.addEventListener("update", addBodyMass);

		if (this.oscillator)
		{
			this.oscillator.oscillatorNode.disconnect();
			this.oscillator.gainNode.disconnect();
		}

		this.oscillator = new Oscillator(0, 200, "square");
		this.oscillator.oscillatorNode.detune.value = 1;
		this.oscillator.start();
		this.oscillator.gainNode.gain.linearRampToValueAtTime(0.3, this.oscillator.audioContext.currentTime + 1);
	}).bind(playground);
}
function mouseMoveHandler(playground: Playground): (event: MouseEvent) => void
{
	let timeOffset = Date.now();
	return (function (this: typeof playground, event: MouseEvent)
	{
		if (this.editedBody)
		{
			const timeDelta = (Date.now() - timeOffset) / 100;
			const newPosition = new DOMPoint(event.offsetX - playground.visualEngine.offset.x, event.offsetY - playground.visualEngine.offset.y);

			this.editedBody.parameters = new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint((newPosition.x - this.editedBody.position.x) / timeDelta, (newPosition.y - this.editedBody.position.y) / timeDelta), new DOMPoint());
			this.editedBody.position = newPosition;

			timeOffset = Date.now();
		}
	}).bind(playground);
}
function mouseUpHandler(playground: Playground): (event: MouseEvent) => void
{
	return (function (this: typeof playground, _event: MouseEvent)
	{
		if (this.editedBody)
		{
			this.physicalEngine.bodyies.push(this.editedBody);
			this.removeEventListener("update", addBodyMass);
			this.editedBody = undefined;

			this.oscillator?.stop();
		}
	}).bind(playground);
}
function resizeHandler(playground: Playground): () => void
{
	return (function (this: typeof playground)
	{
		playground.canvas.width = window.innerWidth;
		playground.canvas.height = window.innerHeight;

		playground.visualEngine.offset = new DOMPoint(playground.canvas.width / 2, playground.canvas.height / 2);
	}).bind(playground);
}

window.onload = () =>
{
	const physicalEngine = new PhysicalEngine([new PhysicalBody(new DOMPoint(0, 0), 20e15, new BodyParameters()), new PhysicalBody(new DOMPoint(200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, 100), new DOMPoint())), new PhysicalBody(new DOMPoint(-200, 0), 1e15, new BodyParameters(new DOMPoint(), new DOMPoint(), new DOMPoint(0, -100), new DOMPoint()))]);
	const canvas = <HTMLCanvasElement>document.getElementById("cnvs");
	const context = canvas?.getContext("2d");

	if (context)
	{
		const visualEngine = new VisualEngine(context, new DOMPoint(), "/planet.svg");
		const playground = new Playground(canvas, context, physicalEngine, visualEngine);
		const onResize = resizeHandler(playground);

		playground.addEventListener("update", playground.fpsCounter.tick.bind(playground.fpsCounter));
		playground.addEventListener("update", draw);

		canvas.addEventListener("mousedown", mouseDownHandler(playground));
		canvas.addEventListener("mousemove", mouseMoveHandler(playground));
		canvas.addEventListener("mouseup", mouseUpHandler(playground));
		window.addEventListener("resize", onResize);

		onResize();
	}
}