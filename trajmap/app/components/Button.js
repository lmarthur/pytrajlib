const Button = (params) => {
  const bg = params.bg ? params.bg : "bg-gray-500";
  const cursor = params.cursor ? params.cursor : "cursor-pointer";

  return (
    <div className={`flex flex-col items-center justify-center rounded ${bg} `}>
      <button
        className={`text-white font-bold py-2 px-4 ${cursor}`}
        onClick={params.onClick}
      >
        {params.name}
      </button>
    </div>
  );
};

export default Button;
